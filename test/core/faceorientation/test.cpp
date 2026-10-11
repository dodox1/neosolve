#include "solvespace.h"

#include "harness.h"

// A workplane on a face takes the axes of the xy, yz or zx plane nearest it,
// so a face of a part lying flat is drawn the way the part is seen.
static void CheckAxes(Test::Helper *helper, Vector n, Vector u, Vector v) {
    Quaternion q = Group::FaceOrientation(n);
    CHECK_TRUE(q.RotationU().Equals(u));
    CHECK_TRUE(q.RotationV().Equals(v));
    CHECK_TRUE(q.RotationN().Equals(n));
}

TEST_CASE(standard_planes) {
    CheckAxes(helper, Vector::From(0, 0, 1),  Vector::From(1, 0, 0),  Vector::From(0, 1, 0));
    CheckAxes(helper, Vector::From(0, 0, -1), Vector::From(-1, 0, 0), Vector::From(0, 1, 0));
    CheckAxes(helper, Vector::From(1, 0, 0),  Vector::From(0, 1, 0),  Vector::From(0, 0, 1));
    CheckAxes(helper, Vector::From(-1, 0, 0), Vector::From(0, -1, 0), Vector::From(0, 0, 1));
    CheckAxes(helper, Vector::From(0, 1, 0),  Vector::From(0, 0, 1),  Vector::From(1, 0, 0));
    CheckAxes(helper, Vector::From(0, -1, 0), Vector::From(0, 0, -1), Vector::From(1, 0, 0));
}

TEST_CASE(oblique_face) {
    Vector n = Vector::From(1, 0, 3).WithMagnitude(1);
    Quaternion q = Group::FaceOrientation(n);
    CHECK_TRUE(q.RotationN().Equals(n));
    CHECK_TRUE(fabs(q.RotationU().Dot(n)) < LENGTH_EPS);
    CHECK_TRUE(q.RotationV().y > 1 - LENGTH_EPS);
}
