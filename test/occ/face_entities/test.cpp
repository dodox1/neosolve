#include "solvespace.h"

#include "harness.h"

#ifdef HAVE_OPENCASCADE

// normal.slvs is extrude_hole with a sketch after it. An edit in the sketch
// regenerates from there only; the extrude's faces must still exist.

static int FacesWithoutEntity(int *faces) {
    Group *g = SK.GetGroup(SS.GW.activeGroup);
    g->GenerateDisplayItems();
    std::vector<uint32_t> seen;
    int missing = 0;
    for(STriangle &t : g->displayMesh.l) {
        uint32_t f = t.meta.face;
        if(f == 0 || std::find(seen.begin(), seen.end(), f) != seen.end()) continue;
        seen.push_back(f);
        hEntity he = { f };
        if(!SK.entity.FindByIdNoOops(he)) missing++;
    }
    *faces = (int)seen.size();
    return missing;
}

TEST_CASE(normal_faces_survive_an_edit_after_the_extrude) {
    CHECK_LOAD("normal.slvs");
    SS.GW.activeGroup = SK.groupOrder[SK.groupOrder.n - 1];
    SK.GetGroup(SS.GW.activeGroup)->Activate();
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    int faces;
    CHECK_TRUE(FacesWithoutEntity(&faces) == 0);
    CHECK_TRUE(faces > 0);

    SS.MarkGroupDirty(SS.GW.activeGroup);
    SS.GenerateAll();
    CHECK_TRUE(FacesWithoutEntity(&faces) == 0);
    CHECK_TRUE(faces > 0);
}

// import.slvs is a box imported in union, then extrude_hole with the extrude
// suppressed; past it the box's faces must still be pickable.
TEST_CASE(import_faces_carried_past_a_suppressed_group) {
    CHECK_LOAD("import.slvs");
    SS.GW.activeGroup = SK.groupOrder[SK.groupOrder.n - 1];
    SK.GetGroup(SS.GW.activeGroup)->Activate();
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    Group *g = SK.GetGroup(SS.GW.activeGroup);
    g->GenerateDisplayItems();
    CHECK_TRUE(g->displayMesh.l.n > 0);
    int untagged = 0;
    for(STriangle &t : g->displayMesh.l) {
        hEntity he = { t.meta.face };
        Entity *e = t.meta.face ? SK.entity.FindByIdNoOops(he) : nullptr;
        if(e == nullptr || !e->IsFace()) untagged++;
    }
    CHECK_TRUE(untagged == 0);
}

#endif
