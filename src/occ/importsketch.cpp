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
#include <ElCLib.hxx>
#include <gp_Pln.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
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

    // Counter-clockwise from s to e. The ends and the radius place the centre;
    // locking it as well would be redundant.
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

    // An ellipse or a spline, as cubics.
    bool Curve(const TopoDS_Edge &e) {
        std::vector<std::array<gp_Pnt, 4>> pieces;
        if(!OccUtil::EdgeAsCubics(e, &pieces)) return false;
        for(const auto &piece : pieces) {
            Vector p[4];
            for(int i = 0; i < 4; i++) {
                p[i] = Place(piece[i]);
            }
            Cubic(p);
        }
        return true;
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

// An arc from vertex vs to vt, counter-clockwise about the circle's axis; a half
// circle as two quarters, whose centre the solver can place.
void SketchArc(EdgeSketcher *fs, Vector normal, const gp_Circ &circ, double from, double to,
               const TopoDS_Vertex &vs, const TopoDS_Vertex &vt) {
    auto at = [&](double a) { return fs->Place(ElCLib::Value(a, circ)); };
    Vector centre = fs->Place(circ.Location()),
           s = fs->Place(BRep_Tool::Pnt(vs)), t = fs->Place(BRep_Tool::Pnt(vt));
    gp_Dir a = circ.Axis().Direction();
    bool against = fs->q.Rotate(Vector::From(a.X(), a.Y(), a.Z())).Dot(normal) < 0;
    if(against) std::swap(s, t);
    if(fabs(to - from - PI) < 0.01) {
        Vector m = at((from + to) / 2);
        fs->Arc(centre, s, m);
        fs->Arc(centre, m, t);
    } else {
        fs->Arc(centre, s, t);
    }
}

// Arcs of one circle that meet end to end, where no other edge meets them,
// become one arc, or the circle when they go all the way round, as a hole split
// into quarters or halves does. Their indices go into done.
void SketchJoinedArcs(EdgeSketcher *fs, Vector normal, const TopoDS_Shape &shape,
                      const TopTools_IndexedMapOfShape &edges, std::vector<bool> *done) {
    TopTools_IndexedDataMapOfShapeListOfShape edgesAt;
    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, edgesAt);

    for(int i = 1; i <= edges.Extent(); i++) {
        BRepAdaptor_Curve ci(TopoDS::Edge(edges(i)));
        if((*done)[i] || ci.GetType() != GeomAbs_Circle || IsFullCircle(ci)) continue;
        gp_Circ circ = ci.Circle();

        // Each arc counter-clockwise about this circle's axis.
        struct Piece { int index; double start, span; TopoDS_Vertex vs, vt; };
        std::vector<Piece> pieces;
        for(int j = i; j <= edges.Extent(); j++) {
            const TopoDS_Edge &e = TopoDS::Edge(edges(j));
            BRepAdaptor_Curve cj(e);
            if((*done)[j] || cj.GetType() != GeomAbs_Circle || IsFullCircle(cj)) continue;
            gp_Circ other = cj.Circle();
            if(other.Location().Distance(circ.Location()) > LENGTH_EPS ||
               fabs(other.Radius() - circ.Radius()) > LENGTH_EPS ||
               !other.Axis().IsParallel(circ.Axis(), Precision::Angular())) continue;
            bool along = other.Axis().Direction().Dot(circ.Axis().Direction()) > 0;
            TopoDS_Vertex v0 = TopExp::FirstVertex(e), v1 = TopExp::LastVertex(e);
            if(!along) std::swap(v0, v1);
            double start = ElCLib::Parameter(circ, BRep_Tool::Pnt(v0));
            pieces.push_back({ j, start, cj.LastParameter() - cj.FirstParameter(), v0, v1 });
        }
        if(pieces.size() < 2) continue;
        std::sort(pieces.begin(), pieces.end(),
                  [](const Piece &a, const Piece &b) { return a.start < b.start; });

        // A piece runs on where it ends in the next one's start and nothing
        // else meets there; with no break they close into the circle.
        size_t n = pieces.size();
        auto runsOn = [&](size_t k) {
            const TopoDS_Vertex &v = pieces[k].vt;
            return v.IsSame(pieces[(k + 1) % n].vs) && edgesAt.FindFromKey(v).Extent() == 2;
        };
        size_t first = n;
        for(size_t k = 0; k < n; k++) {
            if(!runsOn(k)) {
                first = (k + 1) % n;
                break;
            }
        }
        if(first == n) {
            fs->Circle(fs->Place(circ.Location()), circ.Radius());
            for(const Piece &p : pieces) {
                (*done)[p.index] = true;
            }
            continue;
        }
        for(size_t k = 0; k < n;) {
            size_t len = 1;
            while(len < n && runsOn((first + k + len - 1) % n)) len++;
            if(len > 1) {
                const Piece &p = pieces[(first + k) % n],
                            &last = pieces[(first + k + len - 1) % n];
                double span = 0;
                for(size_t m = 0; m < len; m++) {
                    span += pieces[(first + k + m) % n].span;
                    (*done)[pieces[(first + k + m) % n].index] = true;
                }
                SketchArc(fs, normal, circ, p.start, p.start + span, p.vs, last.vt);
            }
            k += len;
        }
    }
}

// Lines, circles and arcs, then everything else as cubics, and points where
// nothing could be drawn. All fixed where they lie.
void SketchEdges(EdgeSketcher *fs, const TopoDS_Shape &shape) {
    Vector normal = SK.GetEntity(SS.GW.ActiveWorkplane())->Normal()->NormalN();

    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    std::vector<bool> done(edges.Extent() + 1, false);
    SketchJoinedArcs(fs, normal, shape, edges, &done);
    std::vector<int> others;
    for(int i = 1; i <= edges.Extent(); i++) {
        if(done[i]) continue;
        const TopoDS_Edge &e = TopoDS::Edge(edges(i));
        BRepAdaptor_Curve c(e);
        if(c.GetType() == GeomAbs_Line) {
            fs->Line(fs->Place(BRep_Tool::Pnt(TopExp::FirstVertex(e))),
                     fs->Place(BRep_Tool::Pnt(TopExp::LastVertex(e))));
        } else if(IsFullCircle(c)) {
            fs->Circle(fs->Place(c.Circle().Location()), c.Circle().Radius());
        } else if(c.GetType() == GeomAbs_Circle) {
            // The parameter runs counter-clockwise about the circle's axis.
            SketchArc(fs, normal, c.Circle(), c.FirstParameter(), c.LastParameter(),
                      TopExp::FirstVertex(e), TopExp::LastVertex(e));
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

// Cut every shown imported solid before the active group with its workplane,
// and draw what the cut leaves into the sketch.
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
        if(g->type != Type::IMPORT_SOLID || g->suppress || !g->visible) continue;
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
