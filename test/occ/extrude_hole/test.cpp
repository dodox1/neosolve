#include "solvespace.h"

#include "harness.h"

#ifdef HAVE_OPENCASCADE

#include "occ/solidmodel.h"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>

// normal.slvs is a sketch with one closed contour inside another, extruded.
// The material is the ring between them, so the solid's volume is the
// difference of the two profile areas times the extrusion height, and every
// wall of the hole is a face of the solid.
//
// Getting that wrong is not loud: BRepBuilderAPI_MakeFace::Add() takes hole
// wires as they come, and if one ends up wound the same way as the outer
// contour, its area is added instead of removed. The face is then invalid,
// the prism built from it is invalid, and the inner contour comes out as
// material of its own -- solvespace/neosolve#7.
static const double VOLUME = 193859.5607;

static Group *LastSolidGroup() {
    for(int i = SK.groupOrder.n - 1; i >= 0; i--) {
        Group *g = SK.group.FindByIdNoOops(SK.groupOrder[i]);
        if(g && g->runningSolidModel && !g->runningSolidModel->shape.IsNull()) {
            return g;
        }
    }
    return nullptr;
}

TEST_CASE(hole_is_subtracted) {
    CHECK_LOAD("normal.slvs");

    Group *g = LastSolidGroup();
    CHECK_TRUE(g != nullptr);
    if(!g) return;

    const TopoDS_Shape &shape = g->runningSolidModel->shape;

    // An invalid solid still renders, badly, so check the topology before the
    // measurement that depends on it.
    BRepCheck_Analyzer analyzer(shape);
    CHECK_TRUE(analyzer.IsValid());

    int solids = 0;
    for(TopExp_Explorer it(shape, TopAbs_SOLID); it.More(); it.Next()) solids++;
    CHECK_TRUE(solids == 1);

    // Volume is taken from the BRep, so it does not move with the chord
    // tolerance or with the OpenCASCADE version, unlike a saved mesh or a
    // rendered image. Add the hole instead of subtracting it and this is out
    // by more than a thousand, so the epsilon can stay loose.
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    if(fabs(props.Mass() - VOLUME) > 1.0) {
        dbp("OCC extrude: volume=%.4f, expected %.4f", props.Mass(), VOLUME);
    }
    CHECK_TRUE(fabs(props.Mass() - VOLUME) <= 1.0);
}

#endif // HAVE_OPENCASCADE
