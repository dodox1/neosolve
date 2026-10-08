#include "solvespace.h"

#include "harness.h"

// A 10 x 20 x 30 box with a hole of radius 2 through its middle, read from
// box.brep and placed 100 along x. Each face of the file has to become an
// entity of the import group, sit where the placement puts it, and be what the
// triangles drawn for that face name, so it can be clicked. A flat face then
// fills a sketch with its points. An imported solid used to offer only the
// corners of its bounding box.

namespace {

Group *GroupOfType(Group::Type type) {
    for(Group &g : SK.group) {
        if(g.type == type) return &g;
    }
    return nullptr;
}

std::vector<Entity *> FacesOf(Group *g) {
    std::vector<Entity *> faces;
    for(Entity &e : SK.entity) {
        if(e.group == g->h && e.IsFace()) faces.push_back(&e);
    }
    return faces;
}

Entity *FaceFacing(Group *g, Vector normal) {
    for(Entity *f : FacesOf(g)) {
        if(g->ImportedFaceIsFlat(f->h) && f->FaceGetNormalNum().Equals(normal)) return f;
    }
    return nullptr;
}

// Where the centroid of the face with this outward normal lies on a box whose
// corner is at origin. The hole is in the middle, so it does not move it.
Vector CentroidOfBoxFace(Vector origin, Vector normal) {
    Vector size = Vector::From(10, 20, 30);
    Vector centre = origin.Plus(size.ScaledBy(0.5));
    return centre.Plus(Vector::From(normal.x * size.x, normal.y * size.y,
                                    normal.z * size.z).ScaledBy(0.5));
}

}

#define CHECK_FACES_AT(g, origin) do {                                          \
    std::vector<Entity *> faces = FacesOf(g);                                   \
    CHECK_TRUE(faces.size() == 7);                                              \
    int flat = 0;                                                               \
    for(Entity *f : faces) {                                                    \
        if(!(g)->ImportedFaceIsFlat(f->h)) continue;                            \
        flat++;                                                                 \
        Vector n = f->FaceGetNormalNum();                                       \
        CHECK_TRUE(f->FaceGetPointNum().Equals(CentroidOfBoxFace(origin, n)));  \
    }                                                                           \
    CHECK_TRUE(flat == 6);                                                      \
} while(0)

// Every triangle the group draws names one of its faces, facing the same way
// where the face is flat.
#define CHECK_MESH_NAMES_FACES(g) do {                                          \
    (g)->GenerateDisplayItems();                                                \
    CHECK_TRUE((g)->displayMesh.l.n > 0);                                       \
    for(const STriangle &tr : (g)->displayMesh.l) {                             \
        Entity *f = SK.entity.FindByIdNoOops(hEntity{ tr.meta.face });          \
        CHECK_TRUE(f != nullptr);                                               \
        CHECK_TRUE(f->IsFace() && f->group == (g)->h);                          \
        if(!(g)->ImportedFaceIsFlat(f->h)) continue;                            \
        CHECK_TRUE(f->FaceGetNormalNum().Dot(tr.Normal().WithMagnitude(1)) >   \
                   1 - LENGTH_EPS);                                             \
    }                                                                           \
} while(0)

TEST_CASE(normal_assembled_faces_are_entities) {
    CHECK_LOAD("normal.slvs");
    Group *g = GroupOfType(Group::Type::IMPORT_SOLID);
    CHECK_TRUE(g != nullptr);
    CHECK_TRUE(g->meshCombine == Group::CombineAs::ASSEMBLE);

    CHECK_FACES_AT(g, Vector::From(100, 0, 0));
    CHECK_MESH_NAMES_FACES(g);
}

TEST_CASE(normal_union_faces_are_entities) {
    CHECK_LOAD("normal.slvs");
    Group *g = GroupOfType(Group::Type::IMPORT_SOLID);
    CHECK_TRUE(g != nullptr);
    g->meshCombine = Group::CombineAs::UNION;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    CHECK_FACES_AT(g, Vector::From(100, 0, 0));
    CHECK_MESH_NAMES_FACES(g);
}

TEST_CASE(normal_faces_follow_the_placement) {
    CHECK_LOAD("normal.slvs");
    Group *g = GroupOfType(Group::Type::IMPORT_SOLID);
    CHECK_TRUE(g != nullptr);
    SK.GetParam(g->h.param(0))->val = 40;
    SK.GetParam(g->h.param(1))->val = -5;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    CHECK_FACES_AT(g, Vector::From(40, -5, 0));
    CHECK_MESH_NAMES_FACES(g);
}

// The bottom face lies in the plane of the sketch. It gives its four corners,
// the centre of the hole and the one vertex OpenCASCADE puts on the hole's rim,
// each locked, and nothing else.
TEST_CASE(normal_flat_face_fills_a_sketch) {
    CHECK_LOAD("normal.slvs");
    Group *g = GroupOfType(Group::Type::IMPORT_SOLID);
    CHECK_TRUE(g != nullptr);
    hGroup hs = SK.groupOrder[SK.groupOrder.n - 1];

    Entity *bottom = FaceFacing(g, Vector::From(0, 0, -1));
    CHECK_TRUE(bottom != nullptr);
    hEntity hbottom = bottom->h;

    SS.GW.activeGroup = hs;
    SK.GetGroup(hs)->Activate();
    g->SketchImportedFace(hbottom);
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    std::vector<Vector> points;
    for(Request &r : SK.request) {
        if(r.group != hs) continue;
        CHECK_TRUE(r.type == Request::Type::DATUM_POINT);
        hEntity hp = r.h.entity(0);
        points.push_back(SK.GetEntity(hp)->PointGetNum());
        bool locked = false;
        for(Constraint &c : SK.constraint) {
            if(c.type == Constraint::Type::WHERE_DRAGGED && c.ptA == hp) locked = true;
        }
        CHECK_TRUE(locked);
    }
    CHECK_TRUE(points.size() == 6);

    Vector centre = Vector::From(105, 10, 0);
    Vector corners[] = { Vector::From(100, 0, 0), Vector::From(110, 0, 0),
                         Vector::From(100, 20, 0), Vector::From(110, 20, 0), centre };
    for(Vector want : corners) {
        bool found = false;
        for(Vector p : points) {
            if(p.Equals(want)) found = true;
        }
        CHECK_TRUE(found);
    }
    int onRim = 0;
    for(Vector p : points) {
        if(fabs(p.Minus(centre).Magnitude() - 2) < LENGTH_EPS) onRim++;
    }
    CHECK_TRUE(onRim == 1);

    Group *s = SK.GetGroup(hs);
    CHECK_TRUE(s->solved.how == SolveResult::OKAY);
    CHECK_TRUE(s->solved.dof == 0);
}

TEST_CASE(normal_curved_face_is_refused) {
    CHECK_LOAD("normal.slvs");
    Group *g = GroupOfType(Group::Type::IMPORT_SOLID);
    CHECK_TRUE(g != nullptr);
    int curved = 0;
    for(Entity *f : FacesOf(g)) {
        if(!g->ImportedFaceIsFlat(f->h)) curved++;
    }
    CHECK_TRUE(curved == 1);
}
