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

// The same fixture, used for what gets drawn rather than what gets built. Its
// outer profile is a rounded rectangle, so it has both real edges and tangent
// joints between the flats and the corners.
//
// Two ways to get this wrong, and both were shipped. Outlines added with
// placeholder normals and no tag are thrown away by everything downstream:
// the 2D exports drop them and the graphics window draws none of them, so an
// OCC solid showed no edges of its own at all. Then, once they are classified,
// normals averaged per face make the facet boundaries of every rounded corner
// read as sharp, which is what the display looks like with the solid's own
// entities hidden.
TEST_CASE(solid_outlines_are_classified) {
    CHECK_LOAD("normal.slvs");

    Group *g = LastSolidGroup();
    CHECK_TRUE(g != nullptr);
    if(!g) return;

    g->GenerateDisplayItems();

    int total = 0, tagged = 0, tangent = 0;
    for(const SOutline &o : g->displayOutlines.l) {
        total++;
        if(o.tag == 0) continue;
        tagged++;
        // Faces this close to parallel meet smoothly; an edge between them is
        // a seam in the tessellation, not something to draw.
        if(o.nl.Dot(o.nr) > 0.95) tangent++;
    }

    if(total == 0 || tagged == 0 || tangent != 0) {
        dbp("OCC outlines: %d total, %d tagged, %d near-tangent",
            total, tagged, tangent);
    }
    CHECK_TRUE(total > 0);
    CHECK_TRUE(tagged > 0);
    CHECK_TRUE(tangent == 0);
}

#endif // HAVE_OPENCASCADE
