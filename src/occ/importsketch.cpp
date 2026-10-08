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

// Fill the active sketch with the points that define a face of the import:
// its vertices and the centres of its circles and arcs, each locked where it
// lies. The sketch refers to nothing in the import afterwards.
void Group::SketchImportedFace(hEntity face) const {
    TopoDS_Face f;
    if(!FaceOf(this, face, &f)) return;

    Vector offset = Vector::From(SK.GetParam(h.param(0))->val,
                                 SK.GetParam(h.param(1))->val,
                                 SK.GetParam(h.param(2))->val);
    Quaternion q = Quaternion::From(SK.GetParam(h.param(3))->val,
                                    SK.GetParam(h.param(4))->val,
                                    SK.GetParam(h.param(5))->val,
                                    SK.GetParam(h.param(6))->val);

    std::vector<Vector> points;
    auto add = [&](const gp_Pnt &p) {
        Vector v = q.Rotate(Vector::From(p.X(), p.Y(), p.Z())).Plus(offset);
        for(const Vector &have : points) {
            if(have.Equals(v)) return;
        }
        points.push_back(v);
    };

    TopTools_IndexedMapOfShape vertices, edges;
    TopExp::MapShapes(f, TopAbs_VERTEX, vertices);
    TopExp::MapShapes(f, TopAbs_EDGE, edges);
    for(int i = 1; i <= vertices.Extent(); i++) {
        add(BRep_Tool::Pnt(TopoDS::Vertex(vertices(i))));
    }
    // A circle in OpenCASCADE has one vertex on its rim and no centre, while
    // the centre of a hole is the first thing anyone wants to fit to.
    for(int i = 1; i <= edges.Extent(); i++) {
        BRepAdaptor_Curve c(TopoDS::Edge(edges(i)));
        if(c.GetType() == GeomAbs_Circle) add(c.Circle().Location());
    }

    for(const Vector &p : points) {
        hRequest hr = SS.GW.AddRequest(Request::Type::DATUM_POINT, /*rememberForUndo=*/false);
        SK.GetEntity(hr.entity(0))->PointForceTo(p);
        Constraint::Constrain(Constraint::Type::WHERE_DRAGGED, hr.entity(0),
                              Entity::NO_ENTITY, Entity::NO_ENTITY);
    }
}

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
