#include "solvespace.h"

#include "harness.h"

#ifdef HAVE_OPENCASCADE

#include "occ/two_sided.h"

// normal.slvs is a fully constrained rectangle on a reference plane, extruded
// with the two-sided subtype, so the solid grows half its thickness each way
// and is symmetric about the sketch.
//
// It used to grow the whole thickness on one side: tbot and ttop are worked out
// correctly, but the OCC path passed BRepPrimAPI_MakePrism only the distance
// between them, and the prism starts wherever the profile sits. Only the shaded
// solid was wrong -- the outline is drawn from generated entities, which sat in
// the right place all along.
// 40 by 20 rectangle through a thickness of 80.
static const double VOLUME = 64000.0;

TEST_CASE(solid_straddles_the_sketch) {
    CHECK_LOAD("normal.slvs");

    Group *g = TwoSidedGroupOf(Group::Type::EXTRUDE);
    CHECK_TRUE(g != nullptr);
    if(!g) return;

    BRepCheck_Analyzer analyzer(g->thisSolidModel->shape);
    CHECK_TRUE(analyzer.IsValid());

    double offPlane = 0.0, volume = 0.0;
    CHECK_TRUE(MeasureOffPlane(g, &offPlane, &volume));

    dbp("extrude two-sided: offPlane=%.6f volume=%.4f", offPlane, volume);

    // The whole point: the centre of mass is in the plane of the sketch. With
    // the solid on one side it is half the thickness away from it.
    CHECK_TRUE(fabs(offPlane) < LENGTH_EPS);

    if(VOLUME > 0.0) {
        CHECK_TRUE(fabs(volume - VOLUME) <= 1.0);
    }
}

#endif // HAVE_OPENCASCADE
