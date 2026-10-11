#include "solvespace.h"

#include "harness.h"

// normal.slvs has two sketches in workplanes of their own. Undoing the
// deletion of the active one must give it back its workplane.
TEST_CASE(delete_group_keeps_workplane) {
    CHECK_LOAD("normal.slvs");
    hGroup hg = SK.groupOrder[SK.groupOrder.n - 1];
    SS.GW.activeGroup = hg;
    SK.GetGroup(hg)->Activate();
    hEntity wp = SK.GetGroup(hg)->activeWorkplane;
    CHECK_TRUE(wp != Entity::FREE_IN_3D);

    // What TextWindow::ScreenDeleteGroup does, and undo, without the menus.
    SS.PushFromCurrentOnto(&SS.undo);
    SS.GW.activeGroup = SK.GetGroup(hg)->PreviousGroup()->h;
    SK.group.RemoveById(hg);
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    SS.GW.ClearSuper();

    SS.PopOntoCurrentFrom(&SS.undo);
    CHECK_TRUE(SS.GW.activeGroup == hg);
    CHECK_TRUE(SK.GetGroup(hg)->activeWorkplane == wp);
}
