//-----------------------------------------------------------------------------
// Taking geometry out of an imported solid into a sketch, where it becomes
// ordinary requests that can be constrained to and built on.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "occ/solidmodel.h"

#ifdef HAVE_OPENCASCADE

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>

namespace SolveSpace {

static bool FaceOf(const Group *g, hEntity face, TopoDS_Face *out) {
    uint32_t index;
    if(!g->ImportedFaceIndex(face, &index)) return false;
    return SolidModelOcc::GetCachedFace(g->linkFile, index, out);
}

bool Group::ImportedFaceIsFlat(hEntity face) const {
    TopoDS_Face f;
    if(!FaceOf(this, face, &f)) return false;
    return BRepAdaptor_Surface(f).GetType() == GeomAbs_Plane;
}

namespace {

// Builds requests in the active sketch, in the shape of the DXF import: the
// first point at a place is locked, any later one there is made coincident
// with it, so the sketch ends up with no freedom and no redundancy.
class FaceSketcher {
public:
    Vector offset;
    Quaternion q;
    std::vector<std::pair<Vector, hEntity>> points;

    Vector Place(const gp_Pnt &p) const {
        return q.Rotate(Vector::From(p.X(), p.Y(), p.Z())).Plus(offset);
    }

    hEntity FindPoint(const Vector &p) const {
        for(const auto &have : points) {
            if(have.first.Equals(p)) return have.second;
        }
        return Entity::NO_ENTITY;
    }

    void ProcessPoint(hEntity he, const Vector &p) {
        SK.GetEntity(he)->PointForceTo(p);
        hEntity have = FindPoint(p);
        if(have != Entity::NO_ENTITY) {
            Constraint::ConstrainCoincident(he, have);
            return;
        }
        Constraint::Constrain(Constraint::Type::WHERE_DRAGGED, he,
                              Entity::NO_ENTITY, Entity::NO_ENTITY);
        points.emplace_back(p, he);
    }

    void Point(const Vector &p) {
        if(FindPoint(p) != Entity::NO_ENTITY) return;
        hRequest hr = SS.GW.AddRequest(Request::Type::DATUM_POINT, /*rememberForUndo=*/false);
        ProcessPoint(hr.entity(0), p);
    }

    void Line(const Vector &p0, const Vector &p1) {
        if(p0.Equals(p1)) return;
        hRequest hr = SS.GW.AddRequest(Request::Type::LINE_SEGMENT, /*rememberForUndo=*/false);
        ProcessPoint(hr.entity(1), p0);
        ProcessPoint(hr.entity(2), p1);
    }

    void Circle(const Vector &c, double r) {
        hRequest hr = SS.GW.AddRequest(Request::Type::CIRCLE, /*rememberForUndo=*/false);
        ProcessPoint(hr.entity(1), c);
        SK.GetEntity(hr.entity(64))->DistanceForceTo(r);
        hConstraint hc = Constraint::Constrain(Constraint::Type::DIAMETER, Entity::NO_ENTITY,
                                               Entity::NO_ENTITY, hr.entity(0));
        SK.GetConstraint(hc)->ModifyToSatisfy();
    }

    // Counter-clockwise from s to e about the workplane normal. The ends are
    // locked and the radius dimensioned, which places the centre; locking the
    // centre as well would be redundant.
    void Arc(const Vector &c, const Vector &s, const Vector &e) {
        hRequest hr = SS.GW.AddRequest(Request::Type::ARC_OF_CIRCLE, /*rememberForUndo=*/false);
        SK.GetEntity(hr.entity(1))->PointForceTo(c);
        ProcessPoint(hr.entity(2), s);
        ProcessPoint(hr.entity(3), e);
        hConstraint hc = Constraint::Constrain(Constraint::Type::DIAMETER, Entity::NO_ENTITY,
                                               Entity::NO_ENTITY, hr.entity(0));
        Constraint *r = SK.GetConstraint(hc);
        r->other = true;
        r->ModifyToSatisfy();
    }
};

bool IsFullCircle(const BRepAdaptor_Curve &c) {
    return c.GetType() == GeomAbs_Circle &&
           fabs(c.LastParameter() - c.FirstParameter() - 2 * PI) < LENGTH_EPS;
}

}

// Fill the active sketch with the geometry of a face of the import: its lines,
// circles and arcs, then a point at every vertex no curve has taken. Everything
// is fixed where it lies, and the sketch refers to nothing in the import
// afterwards.
void Group::SketchImportedFace(hEntity face) const {
    TopoDS_Face f;
    if(!FaceOf(this, face, &f)) return;

    FaceSketcher fs = {};
    fs.offset = Vector::From(SK.GetParam(h.param(0))->val,
                             SK.GetParam(h.param(1))->val,
                             SK.GetParam(h.param(2))->val);
    fs.q = Quaternion::From(SK.GetParam(h.param(3))->val,
                            SK.GetParam(h.param(4))->val,
                            SK.GetParam(h.param(5))->val,
                            SK.GetParam(h.param(6))->val);
    Vector normal = SK.GetEntity(SS.GW.ActiveWorkplane())->Normal()->NormalN();

    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(f, TopAbs_EDGE, edges);
    std::vector<int> others;
    for(int i = 1; i <= edges.Extent(); i++) {
        const TopoDS_Edge &e = TopoDS::Edge(edges(i));
        BRepAdaptor_Curve c(e);
        if(c.GetType() == GeomAbs_Line) {
            fs.Line(fs.Place(BRep_Tool::Pnt(TopExp::FirstVertex(e))),
                    fs.Place(BRep_Tool::Pnt(TopExp::LastVertex(e))));
        } else if(IsFullCircle(c)) {
            fs.Circle(fs.Place(c.Circle().Location()), c.Circle().Radius());
        } else if(c.GetType() == GeomAbs_Circle) {
            // The parameter runs counter-clockwise about the circle's axis.
            Vector s = fs.Place(BRep_Tool::Pnt(TopExp::FirstVertex(e))),
                   t = fs.Place(BRep_Tool::Pnt(TopExp::LastVertex(e))),
                   centre = fs.Place(c.Circle().Location());
            gp_Dir a = c.Circle().Axis().Direction();
            if(fs.q.Rotate(Vector::From(a.X(), a.Y(), a.Z())).Dot(normal) < 0) std::swap(s, t);
            // A half circle has its centre where moving it changes no radius,
            // so the solver cannot tell it is placed; two quarters can.
            if(fabs(c.LastParameter() - c.FirstParameter() - PI) < 0.01) {
                Vector m = fs.Place(c.Value((c.FirstParameter() + c.LastParameter()) / 2));
                fs.Arc(centre, s, m);
                fs.Arc(centre, m, t);
            } else {
                fs.Arc(centre, s, t);
            }
        } else {
            others.push_back(i);
        }
    }

    for(int i : others) {
        const TopoDS_Edge &e = TopoDS::Edge(edges(i));
        fs.Point(fs.Place(BRep_Tool::Pnt(TopExp::FirstVertex(e))));
        fs.Point(fs.Place(BRep_Tool::Pnt(TopExp::LastVertex(e))));
    }
}

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
