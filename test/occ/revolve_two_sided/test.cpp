#include "solvespace.h"

#include "harness.h"

#ifdef HAVE_OPENCASCADE

#include "occ/two_sided.h"

// normal.slvs is a fully constrained rectangle beside an axis on a reference
// plane, revolved with the two-sided subtype, so the sweep runs half its angle
// each way and the solid is symmetric about the sketch.
//
// It used to sweep the whole angle to one side: angles and anglef are worked
// out correctly, but the OCC path passed BRepPrimAPI_MakeRevol only the
// difference between them, and the revolution starts wherever the profile
// sits.
// Measured, not derived: the angle in the group is scaled by four on its
// way in, for reasons the upstream comment does not explain.
static const double VOLUME = 50265.4825;

TEST_CASE(solid_straddles_the_sketch) {
    CHECK_LOAD("normal.slvs");

    Group *g = TwoSidedGroupOf(Group::Type::REVOLVE);
    CHECK_TRUE(g != nullptr);
    if(!g) return;

    BRepCheck_Analyzer analyzer(g->thisSolidModel->shape);
    CHECK_TRUE(analyzer.IsValid());

    double offPlane = 0.0, volume = 0.0;
    CHECK_TRUE(MeasureOffPlane(g, &offPlane, &volume));

    dbp("revolve two-sided: offPlane=%.6f volume=%.4f", offPlane, volume);

    // The whole point: the centre of mass is in the plane of the sketch. With
    // the sweep on one side it is well off it.
    CHECK_TRUE(fabs(offPlane) < LENGTH_EPS);

    if(VOLUME > 0.0) {
        CHECK_TRUE(fabs(volume - VOLUME) <= 1.0);
    }
}

#endif // HAVE_OPENCASCADE
