#include "solvespace.h"
#include "occ/solidmodel.h"

#include "harness.h"

// A cylinder, so that the only edges are two circles and the seams between the
// quadrant faces OpenCASCADE builds it from. Picking one of those circles has
// to name the arcs it is made of, and nothing at the other end.
//
// Matching used to recognise line segments only, so a circle matched nothing
// and fillet and chamfer refused the selection.

namespace {

// The circle entity on the solid at this height, as the user would click it.
hEntity CircleAtHeight(hGroup g, double z) {
    for(Entity &e : SK.entity) {
        if(e.group != g) continue;
        if(e.type != Entity::Type::CIRCLE) continue;
        if(fabs(SK.GetEntity(e.point[0])->PointGetNum().z - z) > LENGTH_EPS) continue;
        return e.h;
    }
    return Entity::NO_ENTITY;
}

std::vector<uint32_t> EdgesPicked(const SolidModelOcc *model, hEntity he, bool *ok) {
    List<GraphicsWindow::Selection> selection = {};
    GraphicsWindow::Selection s = {};
    s.entity = he;
    selection.Add(&s);

    std::vector<uint32_t> edges;
    *ok = model->FindSelectedEdges(&selection, &edges);
    selection.Clear();
    return edges;
}

}

TEST_CASE(normal_picks_the_arcs_of_that_circle) {
    CHECK_LOAD("normal.slvs");

    Group *g = SK.GetGroup(SS.GW.activeGroup);
    CHECK_TRUE(g->runningSolidModel != nullptr);
    CHECK_FALSE(g->runningSolidModel->IsEmpty());

    hEntity top = CircleAtHeight(g->h, 20.0);
    hEntity bottom = CircleAtHeight(g->h, 0.0);
    CHECK_TRUE(top.v != Entity::NO_ENTITY.v);
    CHECK_TRUE(bottom.v != Entity::NO_ENTITY.v);

    bool okTop = false, okBottom = false;
    std::vector<uint32_t> et = EdgesPicked(g->runningSolidModel, top, &okTop);
    std::vector<uint32_t> eb = EdgesPicked(g->runningSolidModel, bottom, &okBottom);

    // A selection that matches nothing returns false, which fillet and chamfer
    // read as "the user asked for something I could not do".
    CHECK_TRUE(okTop);
    CHECK_TRUE(okBottom);

    // Four quadrant faces, so four arcs at each end.
    CHECK_TRUE(et.size() == 4);
    CHECK_TRUE(eb.size() == 4);

    // And the two ends share none of them.
    for(uint32_t a : et) {
        for(uint32_t b : eb) {
            CHECK_TRUE(a != b);
        }
    }
}
