#include "solvespace.h"

#include "harness.h"

// part.brep is the box of import_faces with its corner at (10, 20) rounded to
// r=3, the hole of radius 2 at (5, 10), and a slot with half-round ends of
// radius 1 between x=2.5 and 7.5 around y=4, placed 100 along x. Its bottom face
// lies in the plane of the sketch, so Sketch on Face there has lines, a circle
// and arcs to draw, one of them a half circle.

namespace {

Group *ImportGroup() {
    for(Group &g : SK.group) {
        if(g.type == Group::Type::IMPORT_SOLID) return &g;
    }
    return nullptr;
}

Entity *FaceFacing(Group *g, Vector normal) {
    for(Entity &e : SK.entity) {
        if(e.group != g->h || !e.IsFace()) continue;
        if(g->ImportedFaceIsFlat(e.h) && e.FaceGetNormalNum().Equals(normal)) return &e;
    }
    return nullptr;
}

// The point halfway along an arc, which goes counter-clockwise from its start
// to its end about the sketch normal, +z here.
Vector ArcMiddle(Entity *arc) {
    Vector c = SK.GetEntity(arc->point[0])->PointGetNum(),
           s = SK.GetEntity(arc->point[1])->PointGetNum(),
           e = SK.GetEntity(arc->point[2])->PointGetNum();
    double as = atan2(s.y - c.y, s.x - c.x), ae = atan2(e.y - c.y, e.x - c.x);
    double sweep = ae - as;
    while(sweep <= 0) sweep += 2 * PI;
    double r = s.Minus(c).Magnitude(), m = as + sweep / 2;
    return c.Plus(Vector::From(r * cos(m), r * sin(m), 0));
}

// Turned half a turn about x, the part has y and z negated, and every circle's
// axis points against the sketch normal instead of along it.
void CheckFillsASketch(Test::Helper *helper, bool turned) {
    double sy = turned ? -1 : 1;
    CHECK_LOAD("normal.slvs");
    Group *g = ImportGroup();
    CHECK_TRUE(g != nullptr);
    if(turned) {
        SK.GetParam(g->h.param(3))->val = 0;
        SK.GetParam(g->h.param(4))->val = 1;
        SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    }
    hGroup hs = SK.groupOrder[SK.groupOrder.n - 1];

    Entity *bottom = FaceFacing(g, Vector::From(0, 0, -sy));
    CHECK_TRUE(bottom != nullptr);
    hEntity hbottom = bottom->h;

    SS.GW.activeGroup = hs;
    SK.GetGroup(hs)->Activate();
    g->SketchImportedFace(hbottom);
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    // Everything that was drawn is placed, and nothing twice over.
    Group *s = SK.GetGroup(hs);
    CHECK_TRUE(s->solved.how == SolveResult::OKAY);
    CHECK_TRUE(s->solved.dof == 0);

    int lines = 0, circles = 0, fillets = 0, slotArcs = 0, others = 0;
    for(Request &r : SK.request) {
        if(r.group != hs) continue;
        Entity *e = SK.GetEntity(r.h.entity(0));
        if(r.type == Request::Type::LINE_SEGMENT) {
            lines++;
        } else if(r.type == Request::Type::CIRCLE) {
            circles++;
            CHECK_TRUE(SK.GetEntity(e->point[0])->PointGetNum().Equals(
                Vector::From(105, 10 * sy, 0)));
            CHECK_TRUE(fabs(e->CircleGetRadiusNum() - 2) < LENGTH_EPS);
        } else if(r.type == Request::Type::ARC_OF_CIRCLE) {
            // Each arc has to bulge the way the face does: the rounded corner
            // outwards, the ends of the slot away from its middle. Drawn the
            // wrong way round, an arc would be its own complement.
            double radius = e->CircleGetRadiusNum();
            Vector m = ArcMiddle(e);
            if(fabs(radius - 3) < LENGTH_EPS) {
                fillets++;
                Vector want = Vector::From(107, 17 * sy, 0).Plus(
                    Vector::From(1, sy, 0).WithMagnitude(3));
                CHECK_TRUE(m.Equals(want));
            } else {
                slotArcs++;
                CHECK_TRUE(fabs(radius - 1) < LENGTH_EPS);
                CHECK_TRUE(m.x < 103.5 + LENGTH_EPS || m.x > 106.5 - LENGTH_EPS);
            }
        } else {
            others++;
        }
    }
    CHECK_TRUE(lines == 6);
    CHECK_TRUE(circles == 1);
    CHECK_TRUE(fillets == 1);
    CHECK_TRUE(slotArcs >= 4);
    CHECK_TRUE(others == 0);
}

}

TEST_CASE(normal_flat_face_fills_a_sketch) {
    CheckFillsASketch(helper, /*turned=*/false);
}

TEST_CASE(normal_turned_face_fills_a_sketch) {
    CheckFillsASketch(helper, /*turned=*/true);
}

TEST_CASE(normal_curved_face_is_refused) {
    CHECK_LOAD("normal.slvs");
    Group *g = ImportGroup();
    CHECK_TRUE(g != nullptr);
    int curved = 0;
    for(Entity &e : SK.entity) {
        if(e.group == g->h && e.IsFace() && !g->ImportedFaceIsFlat(e.h)) curved++;
    }
    // The hole, the rounded corner and the two ends of the slot.
    CHECK_TRUE(curved >= 4);
}
