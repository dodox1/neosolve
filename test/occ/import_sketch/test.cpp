#include "solvespace.h"

#include "harness.h"

// part.brep is the box of import_faces with its corner at (10, 20) rounded to
// r=3, the hole of radius 2 at (5, 10), and a slot with half-round ends of
// radius 1 between x=2.5 and 7.5 around y=4, and a hole of radius 0.8 leaning
// 45 degrees towards +x from (3, 16), placed 100 along x. Its bottom face lies in
// the plane of the sketch, so Sketch on Face there has lines, a circle, arcs,
// one of them a half circle, and an ellipse to draw.

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

// How far off the ellipse the leaning hole leaves on the bottom face a point
// is, as a fraction of its size: 0 on it.
double OffEllipse(Vector p, double sy) {
    double u = (p.x - 103) / (0.8 * sqrt(2)), v = (p.y - 16 * sy) / 0.8;
    return fabs(sqrt(u * u + v * v) - 1);
}

// Turned half a turn about x, the part has y and z negated, and every circle's
// axis points against the sketch normal instead of along it. Sectioned, the part
// is moved so the sketch plane cuts it halfway up, where the leaning hole has
// already left through its side.
void CheckFillsASketch(Test::Helper *helper, bool turned, bool section) {
    double sy = turned ? -1 : 1;
    CHECK_LOAD("normal.slvs");
    Group *g = ImportGroup();
    CHECK_TRUE(g != nullptr);
    if(turned) {
        SK.GetParam(g->h.param(3))->val = 0;
        SK.GetParam(g->h.param(4))->val = 1;
    }
    if(section) {
        SK.GetParam(g->h.param(2))->val = -15 * sy;
    }
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    hGroup hs = SK.groupOrder[SK.groupOrder.n - 1];

    Entity *bottom = FaceFacing(g, Vector::From(0, 0, -sy));
    CHECK_TRUE(bottom != nullptr);
    hEntity hbottom = bottom->h;

    SS.GW.activeGroup = hs;
    SK.GetGroup(hs)->Activate();
    if(section) {
        Group::SketchImportedSections(/*rememberForUndo=*/false);
    } else {
        g->SketchImportedFace(hbottom);
    }
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    // Everything that was drawn is placed, and nothing twice over.
    Group *s = SK.GetGroup(hs);
    CHECK_TRUE(s->solved.how == SolveResult::OKAY);
    CHECK_TRUE(s->solved.dof == 0);

    int lines = 0, circles = 0, fillets = 0, slotArcs = 0, cubics = 0, others = 0;
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
        } else if(r.type == Request::Type::CUBIC) {
            // On the ellipse at both ends and halfway along, eight to the turn.
            cubics++;
            Vector p[4];
            for(int i = 0; i < 4; i++) {
                p[i] = SK.GetEntity(e->point[i])->PointGetNum();
            }
            Vector mid = p[0].Plus(p[1].ScaledBy(3)).Plus(p[2].ScaledBy(3)).Plus(p[3])
                             .ScaledBy(1.0 / 8);
            CHECK_TRUE(OffEllipse(p[0], sy) < 1e-5);
            CHECK_TRUE(OffEllipse(p[3], sy) < 1e-5);
            CHECK_TRUE(OffEllipse(mid, sy) < 1e-5);
        } else {
            others++;
        }
    }
    CHECK_TRUE(lines == 6);
    CHECK_TRUE(circles == 1);
    CHECK_TRUE(fillets == 1);
    CHECK_TRUE(slotArcs >= 4);
    CHECK_TRUE(cubics == (section ? 0 : 8));
    CHECK_TRUE(others == 0);
}

}

TEST_CASE(normal_flat_face_fills_a_sketch) {
    CheckFillsASketch(helper, /*turned=*/false, /*section=*/false);
}

TEST_CASE(normal_turned_face_fills_a_sketch) {
    CheckFillsASketch(helper, /*turned=*/true, /*section=*/false);
}

TEST_CASE(normal_section_fills_a_sketch) {
    CheckFillsASketch(helper, /*turned=*/false, /*section=*/true);
}

TEST_CASE(normal_turned_section_fills_a_sketch) {
    CheckFillsASketch(helper, /*turned=*/true, /*section=*/true);
}

// A plane that misses the part adds nothing.
TEST_CASE(normal_section_that_cuts_nothing) {
    CHECK_LOAD("normal.slvs");
    Group *g = ImportGroup();
    CHECK_TRUE(g != nullptr);
    SK.GetParam(g->h.param(2))->val = 100;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    hGroup hs = SK.groupOrder[SK.groupOrder.n - 1];
    SS.GW.activeGroup = hs;
    SK.GetGroup(hs)->Activate();
    Group::SketchImportedSections(/*rememberForUndo=*/false);
    for(Request &r : SK.request) {
        CHECK_TRUE(r.group != hs);
    }
}

TEST_CASE(normal_curved_face_is_refused) {
    CHECK_LOAD("normal.slvs");
    Group *g = ImportGroup();
    CHECK_TRUE(g != nullptr);
    int curved = 0;
    for(Entity &e : SK.entity) {
        if(e.group == g->h && e.IsFace() && !g->ImportedFaceIsFlat(e.h)) curved++;
    }
    // The two holes, the rounded corner and the two ends of the slot.
    CHECK_TRUE(curved >= 5);
}

// Export 2d Section cuts the solid the active group holds. With the part in
// union it is in that solid, and the plane of the sketch cuts it halfway up.
// The section used to be only the flat faces lying in the plane, so a plane
// through the material gave an empty file.
TEST_CASE(normal_section_exports) {
    CHECK_LOAD("normal.slvs");
    Group *g = ImportGroup();
    CHECK_TRUE(g != nullptr);
    g->meshCombine = Group::CombineAs::UNION;
    SK.GetParam(g->h.param(2))->val = -15;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    Group *s = SK.GetGroup(SK.groupOrder[SK.groupOrder.n - 1]);
    Vector n = Vector::From(0, 0, 1);

    SEdgeList el = {};
    SBezierList bl = {};
    CHECK_TRUE(s->MakeOccSectionEdgesInto(n, 0, &el, &bl));
    CHECK_TRUE(el.l.IsEmpty());
    CHECK_TRUE(bl.l.n > 0);
    for(const SBezier &b : bl.l) {
        for(int i = 0; i <= b.deg; i++) {
            Vector p = b.ctrl[i];
            CHECK_TRUE(fabs(p.z) < LENGTH_EPS);
            CHECK_TRUE(p.x > 100 - LENGTH_EPS && p.x < 110 + LENGTH_EPS);
            CHECK_TRUE(p.y > -LENGTH_EPS && p.y < 20 + LENGTH_EPS);
        }
    }
    bl.Clear();

    // Without Beziers it comes as line segments, in the same plane.
    CHECK_TRUE(s->MakeOccSectionEdgesInto(n, 0, &el, nullptr));
    CHECK_TRUE(el.l.n > 0);
    for(const SEdge &e : el.l) {
        CHECK_TRUE(fabs(e.a.z) < LENGTH_EPS && fabs(e.b.z) < LENGTH_EPS);
    }
    el.Clear();

    // A plane lying on a face gives that face's outline, which is all the
    // section used to manage.
    CHECK_TRUE(s->MakeOccSectionEdgesInto(n, -15, &el, &bl));
    CHECK_TRUE(bl.l.n > 0);
    bl.Clear();

    // And a plane above the part cuts nothing.
    CHECK_TRUE(s->MakeOccSectionEdgesInto(n, 100, &el, &bl));
    CHECK_TRUE(el.l.IsEmpty() && bl.l.IsEmpty());
}

// A hidden import is not cut; the section takes what is on screen.
TEST_CASE(normal_section_skips_a_hidden_import) {
    CHECK_LOAD("normal.slvs");
    Group *g = ImportGroup();
    CHECK_TRUE(g != nullptr);
    SK.GetParam(g->h.param(2))->val = -15;
    g->visible = false;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    hGroup hs = SK.groupOrder[SK.groupOrder.n - 1];
    SS.GW.activeGroup = hs;
    SK.GetGroup(hs)->Activate();
    Group::SketchImportedSections(/*rememberForUndo=*/false);
    for(Request &r : SK.request) {
        CHECK_TRUE(r.group != hs);
    }
}
