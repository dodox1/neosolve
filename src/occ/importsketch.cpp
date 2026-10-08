//-----------------------------------------------------------------------------
// Taking geometry out of an imported solid into a sketch, from one of its faces
// or from where a workplane cuts it, as ordinary requests that can be
// constrained to and built on.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "occ/solidmodel.h"

#ifdef HAVE_OPENCASCADE

#include <BRepAlgoAPI_Section.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <GeomConvert_ApproxCurve.hxx>
#include <GeomConvert_BSplineCurveToBezierCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Precision.hxx>
#include <gp_Elips.hxx>
#include <gp_Pln.hxx>
#include <array>
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
class EdgeSketcher {
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

    void Cubic(const Vector *p) {
        hRequest hr = SS.GW.AddRequest(Request::Type::CUBIC, /*rememberForUndo=*/false);
        for(int i = 0; i < 4; i++) {
            ProcessPoint(hr.entity(i + 1), p[i]);
        }
    }

    // The ends meet the neighbouring curves at the face's vertices, not where
    // a conversion happens to put them.
    void Cubics(const TopoDS_Edge &e, std::vector<std::array<Vector, 4>> *pieces) {
        if(pieces->empty()) return;
        pieces->front()[0] = Place(BRep_Tool::Pnt(TopExp::FirstVertex(e)));
        pieces->back()[3] = Place(BRep_Tool::Pnt(TopExp::LastVertex(e)));
        for(const auto &p : *pieces) {
            Cubic(p.data());
        }
    }

    // A cubic is not rational, so an ellipse cannot be held exactly. It is the
    // image of a circle, though, and a circle in 45 degree pieces is good to
    // four parts in a million, a tenth of a micron on a 20 mm ellipse.
    void Ellipse(const BRepAdaptor_Curve &c, std::vector<std::array<Vector, 4>> *pieces) {
        gp_Elips el = c.Ellipse();
        gp_Pnt o = el.Location();
        gp_Vec x = gp_Vec(el.XAxis().Direction()) * el.MajorRadius(),
               y = gp_Vec(el.YAxis().Direction()) * el.MinorRadius();
        auto at = [&](double t) { return o.Translated(x * cos(t) + y * sin(t)); };
        auto along = [&](double t) { return x * -sin(t) + y * cos(t); };

        double t0 = c.FirstParameter(), t1 = c.LastParameter();
        int n = (int)ceil((t1 - t0) / (PI / 4) - 1e-9);
        double dt = (t1 - t0) / n, k = 4.0 / 3 * tan(dt / 4);
        for(int i = 0; i < n; i++) {
            double a = t0 + i * dt, b = a + dt;
            pieces->push_back({{ Place(at(a)), Place(at(a).Translated(along(a) * k)),
                                Place(at(b).Translated(along(b) * -k)), Place(at(b)) }});
        }
    }

    // A spline that is already cubic or less splits into Beziers at its knots
    // exactly; anything else is fitted to a micron.
    bool Spline(const BRepAdaptor_Curve &c, std::vector<std::array<Vector, 4>> *pieces) {
        Handle(Geom_BSplineCurve) bs;
        if(c.GetType() == GeomAbs_BSplineCurve && !c.IsRational() && c.Degree() <= 3) {
            bs = c.BSpline();
        } else {
            Handle(BRepAdaptor_Curve) h = new BRepAdaptor_Curve(c);
            GeomConvert_ApproxCurve approx(h, 1e-3, GeomAbs_C1, 64, 3);
            if(!approx.HasResult()) return false;
            bs = approx.Curve();
        }
        GeomConvert_BSplineCurveToBezierCurve split(bs, c.FirstParameter(), c.LastParameter(),
                                                    Precision::PConfusion());
        for(int i = 1; i <= split.NbArcs(); i++) {
            Handle(Geom_BezierCurve) b = split.Arc(i);
            b->Increase(3);
            pieces->push_back({{ Place(b->Pole(1)), Place(b->Pole(2)),
                                Place(b->Pole(3)), Place(b->Pole(4)) }});
        }
        return true;
    }

    bool Curve(const TopoDS_Edge &e) {
        try {
            BRepAdaptor_Curve c(e);
            std::vector<std::array<Vector, 4>> pieces;
            if(c.GetType() == GeomAbs_Ellipse) {
                Ellipse(c, &pieces);
            } else if(!Spline(c, &pieces)) {
                return false;
            }
            Cubics(e, &pieces);
            return true;
        } catch(const Standard_Failure &) {
            return false;
        }
    }
};

bool IsFullCircle(const BRepAdaptor_Curve &c) {
    return c.GetType() == GeomAbs_Circle &&
           fabs(c.LastParameter() - c.FirstParameter() - 2 * PI) < LENGTH_EPS;
}

// The import's placement, which takes the file's coordinates into the model's.
EdgeSketcher SketcherFor(const Group *g) {
    EdgeSketcher fs = {};
    fs.offset = Vector::From(SK.GetParam(g->h.param(0))->val,
                             SK.GetParam(g->h.param(1))->val,
                             SK.GetParam(g->h.param(2))->val);
    fs.q = Quaternion::From(SK.GetParam(g->h.param(3))->val,
                            SK.GetParam(g->h.param(4))->val,
                            SK.GetParam(g->h.param(5))->val,
                            SK.GetParam(g->h.param(6))->val);
    return fs;
}

// Draw the edges of a shape into the active sketch: lines, circles and arcs,
// then everything else as cubics, and a point at the ends of anything that
// could not be drawn. Everything is fixed where it lies, and the sketch refers
// to nothing in the import afterwards.
void SketchEdges(EdgeSketcher *fs, const TopoDS_Shape &shape) {
    Vector normal = SK.GetEntity(SS.GW.ActiveWorkplane())->Normal()->NormalN();

    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    std::vector<int> others;
    for(int i = 1; i <= edges.Extent(); i++) {
        const TopoDS_Edge &e = TopoDS::Edge(edges(i));
        BRepAdaptor_Curve c(e);
        if(c.GetType() == GeomAbs_Line) {
            fs->Line(fs->Place(BRep_Tool::Pnt(TopExp::FirstVertex(e))),
                     fs->Place(BRep_Tool::Pnt(TopExp::LastVertex(e))));
        } else if(IsFullCircle(c)) {
            fs->Circle(fs->Place(c.Circle().Location()), c.Circle().Radius());
        } else if(c.GetType() == GeomAbs_Circle) {
            // The parameter runs counter-clockwise about the circle's axis.
            Vector s = fs->Place(BRep_Tool::Pnt(TopExp::FirstVertex(e))),
                   t = fs->Place(BRep_Tool::Pnt(TopExp::LastVertex(e))),
                   centre = fs->Place(c.Circle().Location());
            gp_Dir a = c.Circle().Axis().Direction();
            if(fs->q.Rotate(Vector::From(a.X(), a.Y(), a.Z())).Dot(normal) < 0) std::swap(s, t);
            // A half circle has its centre where moving it changes no radius,
            // so the solver cannot tell it is placed; two quarters can.
            if(fabs(c.LastParameter() - c.FirstParameter() - PI) < 0.01) {
                Vector m = fs->Place(c.Value((c.FirstParameter() + c.LastParameter()) / 2));
                fs->Arc(centre, s, m);
                fs->Arc(centre, m, t);
            } else {
                fs->Arc(centre, s, t);
            }
        } else {
            others.push_back(i);
        }
    }

    for(int i : others) {
        const TopoDS_Edge &e = TopoDS::Edge(edges(i));
        if(fs->Curve(e)) continue;
        fs->Point(fs->Place(BRep_Tool::Pnt(TopExp::FirstVertex(e))));
        fs->Point(fs->Place(BRep_Tool::Pnt(TopExp::LastVertex(e))));
    }
}

// Where the active workplane cuts an import's solid, in the file's own
// coordinates, as edges. Empty when it cuts nothing.
TopoDS_Shape SectionOf(const Group *g) {
    TopoDS_Shape shape;
    if(!SolidModelOcc::GetCachedShape(g->linkFile, &shape)) return TopoDS_Shape();

    EdgeSketcher fs = SketcherFor(g);
    Entity *wp = SK.GetEntity(SS.GW.ActiveWorkplane());
    Quaternion back = fs.q.Inverse();
    Vector o = back.Rotate(wp->WorkplaneGetOffset().Minus(fs.offset)),
           n = back.Rotate(wp->Normal()->NormalN());
    try {
        BRepAlgoAPI_Section section(shape, gp_Pln(gp_Pnt(o.x, o.y, o.z), gp_Dir(n.x, n.y, n.z)),
                                    Standard_False);
        section.Approximation(Standard_True);
        section.Build();
        if(!section.IsDone()) return TopoDS_Shape();
        TopTools_IndexedMapOfShape edges;
        TopExp::MapShapes(section.Shape(), TopAbs_EDGE, edges);
        if(edges.Extent() == 0) return TopoDS_Shape();
        return section.Shape();
    } catch(const Standard_Failure &) {
        return TopoDS_Shape();
    }
}

}

void Group::SketchImportedFace(hEntity face) const {
    TopoDS_Face f;
    if(!FaceOf(this, face, &f)) return;

    EdgeSketcher fs = SketcherFor(this);
    SketchEdges(&fs, f);
}

// Cut every imported solid before the active group with its workplane, and
// draw what the cut leaves into the sketch.
void Group::SketchImportedSections(bool rememberForUndo) {
    if(!SS.GW.LockedInWorkplane()) {
        Error(_("Section of Imported Solids cuts with the workplane of the "
                "sketch, so it needs a sketch in a workplane."));
        return;
    }

    std::vector<std::pair<Group *, TopoDS_Shape>> cuts;
    for(hGroup hg : SK.groupOrder) {
        if(hg == SS.GW.activeGroup) break;
        Group *g = SK.GetGroup(hg);
        if(g->type != Type::IMPORT_SOLID || g->suppress) continue;
        TopoDS_Shape cut = SectionOf(g);
        if(!cut.IsNull()) cuts.emplace_back(g, cut);
    }
    if(cuts.empty()) {
        Error(_("The workplane of this sketch does not cut any imported solid."));
        return;
    }

    if(rememberForUndo) SS.UndoRemember();
    for(auto &cut : cuts) {
        EdgeSketcher fs = SketcherFor(cut.first);
        SketchEdges(&fs, cut.second);
    }
}

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
