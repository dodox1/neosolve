#include "solvespace.h"

#include "harness.h"

// A triangulation must cover the polygon exactly: the triangle areas have to
// add up to the polygon's own area, and no triangle may lie outside it. Both
// are checked on random polygons, since the failures found here were on
// ordinary non-convex outlines rather than on any special shape.

namespace {

uint32_t rngState;

double Rnd() {
    rngState = rngState * 1664525u + 1013904223u;
    return (double)(rngState >> 8) / (double)(1u << 24);
}

enum class Shape { CONVEX, NONCONVEX, WITH_HOLES };

void AddDisc(SPolygon *sp, double cx, double cy, double r, int n, bool ccw,
             bool jagged) {
    sp->AddEmptyContour();
    SContour *sc = &sp->l[sp->l.n - 1];
    for(int i = 0; i < n; i++) {
        double t = 2 * PI * i / n;
        if(!ccw) t = -t;
        double rr = jagged ? r * (0.35 + 0.65 * Rnd()) : r;
        sc->AddPoint(Vector::From(cx + rr * cos(t), cy + rr * sin(t), 0));
    }
    sc->AddPoint(sc->l[0].p);
}

void Build(SPolygon *sp, uint32_t seed, Shape shape) {
    // Scramble the seed and warm the generator up; a bare LCG started at a
    // small seed gives nearly the same first value every time.
    rngState = seed * 2654435761u;
    for(int i = 0; i < 4; i++) Rnd();

    *sp = {};
    int outerN = 32 + (int)(Rnd() * 60);
    AddDisc(sp, 0, 0, 50, outerN, /*ccw=*/true,
            /*jagged=*/shape == Shape::NONCONVEX);
    if(shape == Shape::WITH_HOLES) {
        int holes = 1 + (int)(Rnd() * 3);
        for(int i = 0; i < holes; i++) {
            double ang = Rnd() * 2 * PI, rad = Rnd() * 25;
            AddDisc(sp, rad * cos(ang), rad * sin(ang), 3 + Rnd() * 8,
                    6 + (int)(Rnd() * 14), /*ccw=*/false, /*jagged=*/false);
        }
    }
    sp->normal = Vector::From(0, 0, 1);
}

// Triangulate a polygon built from this seed, and say whether the result
// covers it. UvTriangulateInto consumes the polygon, so build it twice.
// A seed whose holes happen to overlap describes no polygon at all, and is
// skipped rather than counted either way.
bool CoversPolygon(uint32_t seed, Shape shape) {
    SPolygon ref = {};
    Build(&ref, seed, shape);
    if(ref.SelfIntersecting(nullptr)) {
        ref.Clear();
        return true;
    }
    ref.FixContourDirections();
    double expected = fabs(ref.SignedArea());

    SPolygon sp = {};
    Build(&sp, seed, shape);
    SSurface srf = SSurface::FromPlane(Vector::From(0, 0, 0),
                                       Vector::From(1, 0, 0),
                                       Vector::From(0, 1, 0));
    SMesh m = {};
    sp.UvTriangulateInto(&m, &srf);

    double area = 0;
    int outside = 0;
    for(int i = 0; i < m.l.n; i++) {
        STriangle *tr = &m.l[i];
        area += tr->Normal().Magnitude() / 2;
        Vector c = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0 / 3);
        if(!ref.ContainsPoint(c)) outside++;
    }

    bool ok = outside == 0 && fabs(area - expected) < 1e-6 * expected;
    m.Clear();
    sp.Clear();
    ref.Clear();
    return ok;
}

}

// CHECK_TRUE reaches for a `helper` that only exists inside a test case.
#define CHECK_COVERS(shape) \
    for(uint32_t seed = 1; seed <= 50; seed++) { \
        CHECK_TRUE(CoversPolygon(seed, shape)); \
    }

TEST_CASE(convex) {
    CHECK_COVERS(Shape::CONVEX);
}

TEST_CASE(non_convex) {
    CHECK_COVERS(Shape::NONCONVEX);
}

TEST_CASE(with_holes) {
    CHECK_COVERS(Shape::WITH_HOLES);
}
