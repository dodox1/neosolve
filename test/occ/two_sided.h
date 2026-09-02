// Shared checks for the two-sided subtypes of the OCC operations. A two-sided
// extrude or revolve is symmetric about the plane of the sketch it grows from,
// so the centre of mass of the solid has to lie in that plane. The OCC calls
// take no starting point and sweep forward from wherever the profile sits, so
// getting this wrong puts the whole solid on one side, which is what this
// measures.
#pragma once

#ifdef HAVE_OPENCASCADE

#include "occ/solidmodel.h"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

namespace {

// The group's own contribution, before it is combined with what came before.
Group *TwoSidedGroupOf(Group::Type type) {
    for(int i = SK.groupOrder.n - 1; i >= 0; i--) {
        Group *g = SK.group.FindByIdNoOops(SK.groupOrder[i]);
        if(g && g->type == type && g->thisSolidModel &&
           !g->thisSolidModel->shape.IsNull()) {
            return g;
        }
    }
    return nullptr;
}

// How far the centre of mass sits off the plane of the sketch the group grew
// from, along that plane's normal, and the volume while we are here.
bool MeasureOffPlane(Group *g, double *offPlane, double *volume) {
    Group *src = SK.group.FindByIdNoOops(g->opA);
    if(!src) return false;

    Entity *wrkpl = SK.entity.FindByIdNoOops(src->h.entity(0));
    if(!wrkpl) return false;
    Vector normal = wrkpl->Normal()->NormalN();
    Vector origin = wrkpl->WorkplaneGetOffset();

    GProp_GProps props;
    BRepGProp::VolumeProperties(g->thisSolidModel->shape, props);
    gp_Pnt com = props.CentreOfMass();

    *volume   = props.Mass();
    *offPlane = Vector::From(com.X(), com.Y(), com.Z()).Minus(origin).Dot(normal);
    return true;
}

} // namespace

#endif // HAVE_OPENCASCADE
