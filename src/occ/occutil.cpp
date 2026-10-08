//-----------------------------------------------------------------------------
// OpenCascade utility functions implementation.
//
//-----------------------------------------------------------------------------

#include "occutil.h"

#ifdef HAVE_OPENCASCADE

#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GeomConvert_ApproxCurve.hxx>
#include <GeomConvert_BSplineCurveToBezierCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <cmath>

namespace SolveSpace {
namespace OccUtil {

// A cubic is not rational, so a conic cannot be held exactly. Every one is the
// image of a circle, though, and a circle's arc has a well known cubic.
static void Conic(gp_Pnt o, gp_Vec x, gp_Vec y, double t0, double t1,
                  std::vector<std::array<gp_Pnt, 4>> *pieces) {
    auto at = [&](double t) { return o.Translated(x * cos(t) + y * sin(t)); };
    auto along = [&](double t) { return x * -sin(t) + y * cos(t); };

    int n = (int)ceil((t1 - t0) / (M_PI / 4) - 1e-9);
    double dt = (t1 - t0) / n, k = 4.0 / 3 * tan(dt / 4);
    for(int i = 0; i < n; i++) {
        double a = t0 + i * dt, b = a + dt;
        pieces->push_back({{ at(a), at(a).Translated(along(a) * k),
                             at(b).Translated(along(b) * -k), at(b) }});
    }
}

static bool Spline(const BRepAdaptor_Curve &c, std::vector<std::array<gp_Pnt, 4>> *pieces) {
    Handle(Geom_BSplineCurve) bs;
    if(c.GetType() == GeomAbs_BSplineCurve && !c.IsRational() && c.Degree() <= 3) {
        bs = c.BSpline();
    } else {
        Handle(BRepAdaptor_Curve) h = new BRepAdaptor_Curve(c);
        GeomConvert_ApproxCurve approx(h, 1e-3, GeomAbs_C1, 64, 3);
        if(!approx.HasResult()) return false;
        bs = approx.Curve();
    }
    GeomConvert_BSplineCurveToBezierCurve split(bs, c.FirstParameter(), c.LastParameter(),
                                                Precision::PConfusion());
    for(int i = 1; i <= split.NbArcs(); i++) {
        Handle(Geom_BezierCurve) b = split.Arc(i);
        b->Increase(3);
        pieces->push_back({{ b->Pole(1), b->Pole(2), b->Pole(3), b->Pole(4) }});
    }
    return true;
}

bool EdgeAsCubics(const TopoDS_Edge &e, std::vector<std::array<gp_Pnt, 4>> *pieces) {
    try {
        BRepAdaptor_Curve c(e);
        double t0 = c.FirstParameter(), t1 = c.LastParameter();
        if(c.GetType() == GeomAbs_Line) {
            return false;
        } else if(c.GetType() == GeomAbs_Circle) {
            gp_Circ ci = c.Circle();
            Conic(ci.Location(), gp_Vec(ci.XAxis().Direction()) * ci.Radius(),
                  gp_Vec(ci.YAxis().Direction()) * ci.Radius(), t0, t1, pieces);
        } else if(c.GetType() == GeomAbs_Ellipse) {
            gp_Elips el = c.Ellipse();
            Conic(el.Location(), gp_Vec(el.XAxis().Direction()) * el.MajorRadius(),
                  gp_Vec(el.YAxis().Direction()) * el.MinorRadius(), t0, t1, pieces);
        } else if(!Spline(c, pieces)) {
            return false;
        }
        if(pieces->empty()) return false;
        // The ends meet the neighbouring curves at the edge's vertices, not
        // where a conversion happens to put them.
        pieces->front()[0] = BRep_Tool::Pnt(TopExp::FirstVertex(e));
        pieces->back()[3] = BRep_Tool::Pnt(TopExp::LastVertex(e));
        return true;
    } catch(const Standard_Failure &) {
        return false;
    }
}

} // namespace OccUtil
} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
