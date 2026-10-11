#include "solvespace.h"

#include "harness.h"

#ifdef HAVE_OPENCASCADE

#include "occ/solidmodel.h"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>

// normal.slvs is the box of import_faces imported to assemble, then the
// extrude_hole sketch and its extrude. An assembled import is a reference:
// drawn and pickable in every later group while shown and not suppressed,
// never part of the solid.

static Group *Import() {
    for(Group &g : SK.group) {
        if(g.type == Group::Type::IMPORT_SOLID) return &g;
    }
    return nullptr;
}

// Every triangle names a face of the import that exists.
static bool PickableAsImport(const SMesh &m, Group *imp) {
    if(m.l.IsEmpty()) return false;
    for(const STriangle &t : m.l) {
        hEntity he = { t.meta.face };
        Entity *e = SK.entity.FindByIdNoOops(he);
        if(e == nullptr || !e->IsFace() || e->group != imp->h) return false;
    }
    return true;
}

static std::vector<Group *> ShownInLast() {
    Group *last = SK.GetGroup(SK.groupOrder[SK.groupOrder.n - 1]);
    std::vector<Group *> refs;
    last->ReferencesShown(&refs);
    return refs;
}

TEST_CASE(normal_reference_shown_past_the_extrude) {
    CHECK_LOAD("normal.slvs");
    Group *imp = Import();
    CHECK_TRUE(imp != nullptr);

    std::vector<Group *> refs = ShownInLast();
    CHECK_TRUE(refs.size() == 1 && refs[0] == imp);
    CHECK_TRUE(PickableAsImport(imp->reference.mesh, imp));

    // Not baked into the extrude's own mesh, so it is not drawn twice.
    Group *last = SK.GetGroup(SK.groupOrder[SK.groupOrder.n - 1]);
    last->GenerateDisplayItems();
    for(const STriangle &t : last->displayMesh.l) {
        hEntity he = { t.meta.face };
        Entity *e = SK.entity.FindByIdNoOops(he);
        CHECK_TRUE(e == nullptr || e->group != imp->h);
    }
}

TEST_CASE(normal_reference_not_in_the_solid) {
    CHECK_LOAD("normal.slvs");
    Group *last = SK.GetGroup(SK.groupOrder[SK.groupOrder.n - 1]);
    CHECK_TRUE(last->runningSolidModel != nullptr);
    GProp_GProps props;
    BRepGProp::VolumeProperties(last->runningSolidModel->shapeAcc, props);
    // extrude_hole's volume alone.
    CHECK_TRUE(fabs(props.Mass() - 193859.5607) <= 1.0);
}

TEST_CASE(normal_reference_hidden_when_not_shown) {
    CHECK_LOAD("normal.slvs");
    Import()->visible = false;
    CHECK_TRUE(ShownInLast().empty());
}

TEST_CASE(normal_reference_hidden_when_suppressed) {
    CHECK_LOAD("normal.slvs");
    Import()->suppress = true;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    CHECK_TRUE(ShownInLast().empty());
}

// What a STEP export writes: the part and, unless suppressed, the reference as
// a body of its own, whether it is shown or not.
static int SolidsExported() {
    Group *last = SK.GetGroup(SK.groupOrder[SK.groupOrder.n - 1]);
    SolidModelOcc m;
    if(!last->ShapeWithReferences(&m)) return 0;
    int n = 0;
    for(TopExp_Explorer ex(m.shapeAcc, TopAbs_SOLID); ex.More(); ex.Next()) n++;
    return n;
}

TEST_CASE(normal_reference_exported_beside_the_part) {
    CHECK_LOAD("normal.slvs");
    CHECK_TRUE(SolidsExported() == 2);
    Import()->visible = false;
    CHECK_TRUE(SolidsExported() == 2);
}

TEST_CASE(normal_suppressed_reference_not_exported) {
    CHECK_LOAD("normal.slvs");
    Import()->suppress = true;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    CHECK_TRUE(SolidsExported() == 1);
}

#endif
