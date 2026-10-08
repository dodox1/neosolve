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
};

bool IsFullCircle(const BRepAdaptor_Curve &c) {
    return c.GetType() == GeomAbs_Circle &&
           fabs(c.LastParameter() - c.FirstParameter() - 2 * PI) < LENGTH_EPS;
}

}

// Fill the active sketch with the geometry of a face of the import: its lines
// and circles, then a point at every vertex no curve has taken and at the
// centre of every arc. Everything is locked where it lies, and the sketch
// refers to nothing in the import afterwards.
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
        } else {
            others.push_back(i);
        }
    }

    for(int i : others) {
        const TopoDS_Edge &e = TopoDS::Edge(edges(i));
        fs.Point(fs.Place(BRep_Tool::Pnt(TopExp::FirstVertex(e))));
        fs.Point(fs.Place(BRep_Tool::Pnt(TopExp::LastVertex(e))));
        BRepAdaptor_Curve c(e);
        // An arc has no centre of its own, and it is what anyone fits to first.
        if(c.GetType() == GeomAbs_Circle) fs.Point(fs.Place(c.Circle().Location()));
    }
}

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
