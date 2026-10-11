//-----------------------------------------------------------------------------
// Routines to generate our watertight brep shells from the operations
// and entities specified by the user in each group; templated to work either
// on an SShell of ratpoly surfaces or on an SMesh of triangles.
//
// Copyright 2008-2013 Jonathan Westhues.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "profiler.h"
#include <cstdarg>

#ifdef HAVE_OPENCASCADE
#include "occ/solidmodel.h"
#include "occ/facebuilder.h"
#include "occ/occutil.h"
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <GeomAbs_Shape.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>
#include <gp_Quaternion.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <list>
#endif

namespace SolveSpace {

void Group::AssembleLoops(bool *allClosed,
                          bool *allCoplanar,
                          bool *allNonZeroLen)
{
    SBezierList sbl = {};

    int i;
    for(auto &e : SK.entity) {
        if(e.group != h)
            continue;
        if(e.construction)
            continue;
        if(e.forceHidden)
            continue;

        e.GenerateBezierCurves(&sbl);
    }

    SBezier *sb;
    *allNonZeroLen = true;
    for(sb = sbl.l.First(); sb; sb = sbl.l.NextAfter(sb)) {
        for(i = 1; i <= sb->deg; i++) {
            if(!(sb->ctrl[i]).Equals(sb->ctrl[0])) {
                break;
            }
        }
        if(i > sb->deg) {
            // This is a zero-length edge.
            *allNonZeroLen = false;
            polyError.errorPointAt = sb->ctrl[0];
            goto done;
        }
    }

    // Try to assemble all these Beziers into loops. The closed loops go into
    // bezierLoops, with the outer loops grouped with their holes. The
    // leftovers, if any, go in bezierOpens.
    bezierLoops.FindOuterFacesFrom(&sbl, &polyLoops, NULL,
                                   SS.ChordTolMm(),
                                   allClosed, &(polyError.notClosedAt),
                                   allCoplanar, &(polyError.errorPointAt),
                                   &bezierOpens);
    done:
    sbl.Clear();
}

void Group::GenerateLoops() {
    polyLoops.Clear();
    bezierLoops.Clear();
    bezierOpens.Clear();

    if(type == Type::DRAWING_3D || type == Type::DRAWING_WORKPLANE ||
       type == Type::ROTATE || type == Type::TRANSLATE || type == Type::MIRROR ||
       type == Type::LINKED)
    {
        bool allClosed = false, allCoplanar = false, allNonZeroLen = false;
        AssembleLoops(&allClosed, &allCoplanar, &allNonZeroLen);
        if(!allNonZeroLen) {
            polyError.how = PolyError::ZERO_LEN_EDGE;
        } else if(!allCoplanar) {
            polyError.how = PolyError::NOT_COPLANAR;
        } else if(!allClosed) {
            polyError.how = PolyError::NOT_CLOSED;
        } else {
            polyError.how = PolyError::GOOD;
            // The self-intersecting check is kind of slow, so don't run it
            // unless requested.
            if(SS.checkClosedContour) {
                if(polyLoops.SelfIntersecting(&(polyError.errorPointAt))) {
                    polyError.how = PolyError::SELF_INTERSECTING;
                }
            }
        }
    }
}

void SShell::RemapFaces(Group *g, int remap) {
    for(SSurface &ss : surface){
        hEntity face = { ss.face };
        if(face == Entity::NO_ENTITY) continue;

        face = g->Remap(face, remap);
        ss.face = face.v;
    }
}

void SMesh::RemapFaces(Group *g, int remap) {
    STriangle *tr;
    for(tr = l.First(); tr; tr = l.NextAfter(tr)) {
        hEntity face = { tr->meta.face };
        if(face == Entity::NO_ENTITY) continue;

        face = g->Remap(face, remap);
        tr->meta.face = face.v;
    }
}

template<class T>
void Group::GenerateForStepAndRepeat(T *steps, T *outs, Group::CombineAs forWhat) {
    PROFILE_SCOPE("StepAndRepeat");
    int n = (int)valA, a0 = 0;
    if(subtype == Subtype::ONE_SIDED && skipFirst) {
        a0++; n++;
    }

    int a;
    // create all the transformed copies
    std::vector <T> transd(n);
    std::vector <T> workA(n);
    workA[0] = {};
    // first generate a shell/mesh with each transformed copy
#pragma omp parallel for
    for(a = a0; a < n; a++) {
        transd[a] = {};
        workA[a] = {};
        int ap = a*2 - (subtype == Subtype::ONE_SIDED ? 0 : (n-1));

        if(type == Type::TRANSLATE) {
            Vector trans = Vector::From(h.param(0), h.param(1), h.param(2));
            trans = trans.ScaledBy(ap);
            transd[a].MakeFromTransformationOf(steps,
                trans, Quaternion::IDENTITY, 1.0);
        } else {
            Vector trans = Vector::From(h.param(0), h.param(1), h.param(2));
            double theta = ap * SK.GetParam(h.param(3))->val;
            double c = cos(theta), s = sin(theta);
            Vector axis = Vector::From(h.param(4), h.param(5), h.param(6));
            Quaternion q = Quaternion::From(c, s*axis.x, s*axis.y, s*axis.z);
            // Rotation is centered at t; so A(x - t) + t = Ax + (t - At)
            transd[a].MakeFromTransformationOf(steps,
                trans.Minus(q.Rotate(trans)), q, 1.0);
        }
    }
    for(a = a0; a < n; a++) {
        // We need to rewrite any plane face entities to the transformed ones.
        int remap = (a == (n - 1)) ? REMAP_LAST : a;
        transd[a].RemapFaces(this, remap);
    }

    std::vector<T> *soFar = &transd;
    std::vector<T> *scratch = &workA;
    // do the boolean operations on pairs of equal size
    while(n > 1) {
        for(a = 0; a < n; a+=2) {
            scratch->at(a/2).Clear();
            // combine a pair of shells
            if((a==0) && (a0==1)) { // if the first was skipped just copy the 2nd
                scratch->at(a/2).MakeFromCopyOf(&(soFar->at(a+1)));
                (soFar->at(a+1)).Clear();
                a0 = 0;
            } else if (a == n-1) { // for an odd number just copy the last one
                scratch->at(a/2).MakeFromCopyOf(&(soFar->at(a)));
                (soFar->at(a)).Clear();
            } else if(forWhat == CombineAs::ASSEMBLE) {
                scratch->at(a/2).MakeFromAssemblyOf(&(soFar->at(a)), &(soFar->at(a+1)));
                (soFar->at(a)).Clear();
                (soFar->at(a+1)).Clear();
            } else {
                scratch->at(a/2).MakeFromUnionOf(&(soFar->at(a)), &(soFar->at(a+1)));
                (soFar->at(a)).Clear();
                (soFar->at(a+1)).Clear();
            }
        }
        swap(scratch, soFar);
        n = (n+1)/2;
    }
    outs->Clear();
    *outs = soFar->at(0);
}

template<class T>
void Group::GenerateForBoolean(T *prevs, T *thiss, T *outs, Group::CombineAs how) {
    PROFILE_SCOPE("BooleanOp");
    // If this group contributes no new mesh, then our running mesh is the
    // same as last time, no combining required. Likewise if we have a mesh
    // but it's suppressed.
    if(thiss->IsEmpty() || suppress) {
        outs->MakeFromCopyOf(prevs);
        return;
    }

    // So our group's shell appears in thisShell. Combine this with the
    // previous group's shell, using the requested operation.
    switch(how) {
        case CombineAs::UNION:
            outs->MakeFromUnionOf(prevs, thiss);
            break;

        case CombineAs::DIFFERENCE:
            outs->MakeFromDifferenceOf(prevs, thiss);
            break;

        case CombineAs::INTERSECTION:
            outs->MakeFromIntersectionOf(prevs, thiss);
            break;

        case CombineAs::ASSEMBLE:
            outs->MakeFromAssemblyOf(prevs, thiss);
            break;
    }
}

void Group::OccFailed(const char *fmt, ...) {
    va_list va;

    va_start(va, fmt);
    int size = vsnprintf(NULL, 0, fmt, va);
    va_end(va);
    if(size < 0) return;

    std::string msg;
    msg.resize(size + 1);
    va_start(va, fmt);
    vsnprintf(&msg[0], size + 1, fmt, va);
    va_end(va);
    msg.resize(size);

    dbp("%s", msg.c_str());
    // Keep the first one: the ones after it tend to be its consequences.
    if(occError.empty()) occError = msg;
}

void Group::GenerateShellAndMesh() {
    PROFILE_FUNCTION();
    bool prevBooleanFailed = booleanFailed;
    booleanFailed = false;
    occError.clear();
    occFaceEntities = false;

    Group *srcg = this;

    thisShell.Clear();
    thisMesh.Clear();
    runningShell.Clear();
    runningMesh.Clear();

#ifdef HAVE_OPENCASCADE
    if(!thisSolidModel) {
        thisSolidModel = new SolidModelOcc();
    }
    if(!runningSolidModel) {
        runningSolidModel = new SolidModelOcc();
    }
    // Don't clear imported solids - they are cached
    if(type != Type::IMPORT_SOLID) {
        thisSolidModel->Clear();
    }
    runningSolidModel->Clear();
#endif

    // Don't attempt a lathe or extrusion unless the source section is good:
    // planar and not self-intersecting.
    bool haveSrc = true;
    if(type == Type::EXTRUDE || type == Type::LATHE || type == Type::REVOLVE) {
        Group *src = SK.GetGroup(opA);
        if(src->polyError.how != PolyError::GOOD) {
            haveSrc = false;
        }
    }

    if(type == Type::TRANSLATE || type == Type::ROTATE) {
        // A step and repeat gets merged against the group's previous group,
        // not our own previous group.
        srcg = SK.GetGroup(opA);

        if(!srcg->suppress) {
#ifdef HAVE_OPENCASCADE
            // Check if source group has OCC solid model - if so, use OCC transformations
            bool srcHasOccShape = srcg->thisSolidModel &&
                                  !srcg->thisSolidModel->shape.IsNull();
            if(srcHasOccShape) {
                // Use OCC-based step and repeat
                int n = (int)valA, a0 = 0;
                if(subtype == Subtype::ONE_SIDED && skipFirst) {
                    a0++; n++;
                }

                PROFILE_SCOPE("OccStepRepeat");
                TopTools_ListOfShape copies;
                for(int a = a0; a < n; a++) {
                    int ap = a*2 - (subtype == Subtype::ONE_SIDED ? 0 : (n-1));

                    gp_Trsf trsf;
                    if(type == Type::TRANSLATE) {
                        Vector trans = Vector::From(h.param(0), h.param(1), h.param(2));
                        trans = trans.ScaledBy(ap);
                        trsf.SetTranslation(gp_Vec(trans.x, trans.y, trans.z));
                    } else { // ROTATE
                        Vector trans = Vector::From(h.param(0), h.param(1), h.param(2));
                        double theta = ap * SK.GetParam(h.param(3))->val;
                        Vector axis = Vector::From(h.param(4), h.param(5), h.param(6));

                        // Build rotation quaternion
                        double c = cos(theta), s = sin(theta);
                        gp_Quaternion occQuat(s*axis.x, s*axis.y, s*axis.z, c);
                        trsf.SetRotation(occQuat);

                        // Rotation is centered at trans; so A(x - t) + t = Ax + (t - At)
                        gp_Pnt transP(trans.x, trans.y, trans.z);
                        gp_Pnt rotatedP = transP.Transformed(trsf);
                        gp_Vec offset(transP.X() - rotatedP.X(),
                                      transP.Y() - rotatedP.Y(),
                                      transP.Z() - rotatedP.Z());
                        trsf.SetTranslationPart(offset);
                    }

                    // Transform the source shape
                    BRepBuilderAPI_Transform transformer(srcg->thisSolidModel->shape, trsf, Standard_True);
                    if(!transformer.IsDone()) {
                        continue;
                    }
                    copies.Append(transformer.Shape());
                }

                // One fuse taking every copy at once, for the same reason as
                // the extrude path above.
                if(copies.Extent() == 1) {
                    thisSolidModel->shape = copies.First();
                } else if(copies.Extent() > 1) {
                    try {
                        TopTools_ListOfShape arguments;
                        arguments.Append(copies.First());
                        copies.RemoveFirst();

                        BRepAlgoAPI_Fuse fuser;
                        fuser.SetArguments(arguments);
                        fuser.SetTools(copies);
                        fuser.SetRunParallel(Standard_True);
                        fuser.Build();
                        if(fuser.IsDone()) {
                            thisSolidModel->shape = fuser.Shape();
                        }
                    } catch(const Standard_Failure &e) {
                        OccFailed("OCC step-and-repeat fuse failed: %s", e.GetMessageString());
                    }
                }
            } else
#endif
            // Fall through to shell/mesh path when OCC not available or source has no OCC shape
            if(!IsForcedToMesh()) {
                GenerateForStepAndRepeat<SShell>(&(srcg->thisShell), &thisShell, srcg->meshCombine);
            } else {
                SMesh prevm = {};
                prevm.MakeFromCopyOf(&srcg->thisMesh);
                srcg->thisShell.TriangulateInto(&prevm);
                GenerateForStepAndRepeat<SMesh> (&prevm, &thisMesh, srcg->meshCombine);
            }
        }
    } else if(type == Type::MIRROR) {
        // Mirror gets its geometry from the source group
        srcg = SK.GetGroup(opA);
        if(!srcg->suppress) {
            // Get mirror transformation parameters
            Vector mirrorOrigin = Vector::From(h.param(0), h.param(1), h.param(2));
            Vector mirrorNormal = Vector::From(h.param(3), h.param(4), h.param(5)).WithMagnitude(1);

            // Helper lambda to mirror a point about the plane
            auto mirrorPoint = [&](Vector p) -> Vector {
                double d = (p.Minus(mirrorOrigin)).Dot(mirrorNormal);
                return p.Minus(mirrorNormal.ScaledBy(2 * d));
            };

#ifdef HAVE_OPENCASCADE
            // Check if source group has OCC solid model
            bool srcHasOccShape = srcg->thisSolidModel &&
                                  !srcg->thisSolidModel->shape.IsNull();
            bool occMirrorSucceeded = false;
            if(srcHasOccShape) {
                try {
                    // Use OCC mirror transformation
                    // Mirror plane passes through mirrorOrigin with normal mirrorNormal
                    gp_Pnt occOrigin(mirrorOrigin.x, mirrorOrigin.y, mirrorOrigin.z);
                    gp_Dir occNormal(mirrorNormal.x, mirrorNormal.y, mirrorNormal.z);
                    gp_Ax2 mirrorAxis(occOrigin, occNormal);

                    gp_Trsf trsf;
                    trsf.SetMirror(mirrorAxis);

                    BRepBuilderAPI_Transform transformer(srcg->thisSolidModel->shape, trsf, Standard_True);
                    if(transformer.IsDone()) {
                        thisSolidModel->shape = transformer.Shape();
                        occMirrorSucceeded = true;
                    }
                } catch(const Standard_Failure &e) {
                    OccFailed("OCC mirror failed: %s", e.GetMessageString());
                }
            }
            // Fall back to mesh path if OCC not available or failed
            if(!occMirrorSucceeded)
#endif
            // Fall through to shell/mesh path
            if(!IsForcedToMesh()) {
                // Mirror the shell by transforming it with scale -1 and appropriate rotation
                // For now, triangulate and use mesh path (shell mirroring is complex)
                SMesh srcMesh = {};
                srcMesh.MakeFromCopyOf(&srcg->thisMesh);
                srcg->thisShell.TriangulateInto(&srcMesh);

                // Mirror each triangle
                for(STriangle &tr : srcMesh.l) {
                    STriangle mirrored = tr;
                    mirrored.a = mirrorPoint(tr.a);
                    mirrored.b = mirrorPoint(tr.b);
                    mirrored.c = mirrorPoint(tr.c);
                    std::swap(mirrored.b, mirrored.c);
                    mirrored.an = mirrorPoint(tr.a.Plus(tr.an)).Minus(mirrored.a);
                    mirrored.bn = mirrorPoint(tr.b.Plus(tr.bn)).Minus(mirrored.b);
                    mirrored.cn = mirrorPoint(tr.c.Plus(tr.cn)).Minus(mirrored.c);
                    thisMesh.AddTriangle(&mirrored);
                }
                srcMesh.Clear();
                // Force mesh mode for the combination since we only have mesh
                forceToMesh = true;
            } else {
                // Mirror the mesh triangles
                SMesh srcMesh = {};
                srcMesh.MakeFromCopyOf(&srcg->thisMesh);
                srcg->thisShell.TriangulateInto(&srcMesh);

                for(STriangle &tr : srcMesh.l) {
                    STriangle mirrored = tr;
                    mirrored.a = mirrorPoint(tr.a);
                    mirrored.b = mirrorPoint(tr.b);
                    mirrored.c = mirrorPoint(tr.c);
                    std::swap(mirrored.b, mirrored.c);
                    mirrored.an = mirrorPoint(tr.a.Plus(tr.an)).Minus(mirrored.a);
                    mirrored.bn = mirrorPoint(tr.b.Plus(tr.bn)).Minus(mirrored.b);
                    mirrored.cn = mirrorPoint(tr.c.Plus(tr.cn)).Minus(mirrored.c);
                    thisMesh.AddTriangle(&mirrored);
                }
                srcMesh.Clear();
            }
        }
    } else if(type == Type::EXTRUDE && haveSrc) {
        Group *src = SK.GetGroup(opA);

        // Ensure the source group has loops generated (needed for OCC extrusion)
        // This is especially important in export mode where GenerateLoops is not called separately
        if(src->bezierLoops.l.IsEmpty()) {
            src->GenerateLoops();
        }

        Vector translate = Vector::From(h.param(0), h.param(1), h.param(2));

        Vector tbot, ttop;
        if(subtype == Subtype::ONE_SIDED || subtype == Subtype::ONE_SKEWED) {
            tbot = Vector::From(0, 0, 0); ttop = translate.ScaledBy(2);
        } else {
            tbot = translate.ScaledBy(-1); ttop = translate.ScaledBy(1);
        }

#ifdef HAVE_OPENCASCADE
        // Use OCC for extrusion
        SBezierLoopSetSet *sblss = &(src->bezierLoops);
        SBezierLoopSet *sbls;
        TopTools_ListOfShape prisms;
        for(sbls = sblss->l.First(); sbls; sbls = sblss->l.NextAfter(sbls)) {
            // Convert sketch profile to OCC face
            FaceBuilder fb = FaceBuilder::FromBezierLoopSet(sbls);
            if(!fb.IsValid()) continue;

            // Create extrusion direction vector
            Vector extDir = ttop.Minus(tbot);
            gp_Vec occDir = OccUtil::ToOccVec(extDir);

            try {
                // MakePrism sweeps from wherever the profile sits and takes no
                // starting point, so move the profile to tbot first.
                TopoDS_Shape profile = fb.GetFaces();
                if(!tbot.Equals(Vector::From(0, 0, 0))) {
                    gp_Trsf toStart;
                    toStart.SetTranslation(OccUtil::ToOccVec(tbot));
                    profile = BRepBuilderAPI_Transform(profile, toStart,
                                                       Standard_True).Shape();
                }

                // Create the prism (extrusion)
                BRepPrimAPI_MakePrism prism(profile, occDir);
                if(prism.IsDone()) {
                    TopoDS_Shape result = prism.Shape();

                    // Check if we need to reverse the solid (negative volume means inside-out)
                    GProp_GProps props;
                    BRepGProp::VolumeProperties(result, props);
                    double volume = props.Mass();
                    if(volume < 0) {
                        result.Reverse();
                    }

                    prisms.Append(result);
                }
            } catch(const Standard_Failure &e) {
                // OCC extrusion failed - silently fall through to native NURBS
            }
        }

        // One fuse taking every profile at once, rather than one fuse per
        // profile onto the shape so far. Fusing as we go makes OCC intersect
        // everything accumulated against each new profile, so the cost grows
        // with the square of their number, and a sketch patterned into a grid
        // arrives here as a single extrude of hundreds of profiles.
        if(prisms.Extent() == 1) {
            thisSolidModel->shape = prisms.First();
        } else if(prisms.Extent() > 1) {
            try {
                TopTools_ListOfShape arguments;
                arguments.Append(prisms.First());
                prisms.RemoveFirst();

                BRepAlgoAPI_Fuse fuser;
                fuser.SetArguments(arguments);
                fuser.SetTools(prisms);
                fuser.SetRunParallel(Standard_True);
                fuser.Build();
                if(fuser.IsDone()) {
                    thisSolidModel->shape = fuser.Shape();
                }
            } catch(const Standard_Failure &e) {
                OccFailed("OCC extrude fuse failed: %s", e.GetMessageString());
            }
        }
#else
        SBezierLoopSetSet *sblss = &(src->bezierLoops);
        SBezierLoopSet *sbls;
        for(sbls = sblss->l.First(); sbls; sbls = sblss->l.NextAfter(sbls)) {
            int is = thisShell.surface.n;
            // Extrude this outer contour (plus its inner contours, if present)
            thisShell.MakeFromExtrusionOf(sbls, tbot, ttop, color);

            // And for any plane faces, annotate the model with the entity for
            // that face, so that the user can select them with the mouse.
            Vector onOrig = sbls->point;
            int i;
            // Not using range-for here because we're starting at a different place and using
            // indices for meaning.
            for(i = is; i < thisShell.surface.n; i++) {
                SSurface *ss = &(thisShell.surface[i]);
                hEntity face = Entity::NO_ENTITY;

                Vector p = ss->PointAt(0, 0),
                       n = ss->NormalAt(0, 0).WithMagnitude(1);
                double d = n.Dot(p);

                if(i == is || i == (is + 1)) {
                    // These are the top and bottom of the shell.
                    if(fabs((onOrig.Plus(ttop)).Dot(n) - d) < LENGTH_EPS) {
                        face = Remap(Entity::NO_ENTITY, REMAP_TOP);
                        ss->face = face.v;
                    }
                    if(fabs((onOrig.Plus(tbot)).Dot(n) - d) < LENGTH_EPS) {
                        face = Remap(Entity::NO_ENTITY, REMAP_BOTTOM);
                        ss->face = face.v;
                    }
                    continue;
                }

                // So these are the sides
                if(ss->degm != 1 || ss->degn != 1) continue;

                for(Entity &e : SK.entity) {
                    if(e.group != opA) continue;
                    if(e.type != Entity::Type::LINE_SEGMENT) continue;

                    Vector a = SK.GetEntity(e.point[0])->PointGetNum(),
                           b = SK.GetEntity(e.point[1])->PointGetNum();
                    a = a.Plus(ttop);
                    b = b.Plus(ttop);
                    // Could get taken backwards, so check all cases.
                    if((a.Equals(ss->ctrl[0][0]) && b.Equals(ss->ctrl[1][0])) ||
                       (b.Equals(ss->ctrl[0][0]) && a.Equals(ss->ctrl[1][0])) ||
                       (a.Equals(ss->ctrl[0][1]) && b.Equals(ss->ctrl[1][1])) ||
                       (b.Equals(ss->ctrl[0][1]) && a.Equals(ss->ctrl[1][1])))
                    {
                        face = Remap(e.h, REMAP_LINE_TO_FACE);
                        ss->face = face.v;
                        break;
                    }
                }
            }
        }
#endif
    } else if(type == Type::LATHE && haveSrc) {
        Group *src = SK.GetGroup(opA);

        // Ensure the source group has loops generated (needed for OCC lathe)
        if(src->bezierLoops.l.IsEmpty()) {
            src->GenerateLoops();
        }

        Vector pt   = SK.GetEntity(predef.origin)->PointGetNum(),
               axis = SK.GetEntity(predef.entityB)->VectorGetNum();
        axis = axis.WithMagnitude(1);

#ifdef HAVE_OPENCASCADE
        // Use OCC for full revolution (lathe)
        SBezierLoopSetSet *sblss = &(src->bezierLoops);
        SBezierLoopSet *sbls;
        for(sbls = sblss->l.First(); sbls; sbls = sblss->l.NextAfter(sbls)) {
            FaceBuilder fb = FaceBuilder::FromBezierLoopSet(sbls);
            if(!fb.IsValid()) continue;

            // Create rotation axis
            gp_Ax1 occAxis(OccUtil::ToOccPoint(pt), OccUtil::ToOccDir(axis));

            try {
                // Full 360° revolution
                BRepPrimAPI_MakeRevol revol(fb.GetFaces(), occAxis);
                if(revol.IsDone()) {
                    if(thisSolidModel->shape.IsNull()) {
                        thisSolidModel->shape = revol.Shape();
                    } else {
                        thisSolidModel->shape = BRepAlgoAPI_Fuse(
                            thisSolidModel->shape, revol.Shape()).Shape();
                    }
                }
            } catch(const Standard_Failure &e) {
                OccFailed("OCC lathe failed: %s", e.GetMessageString());
            }
        }
#else
        SBezierLoopSetSet *sblss = &(src->bezierLoops);
        SBezierLoopSet *sbls;
        for(sbls = sblss->l.First(); sbls; sbls = sblss->l.NextAfter(sbls)) {
            thisShell.MakeFromRevolutionOf(sbls, pt, axis, color, this);
        }
#endif
    } else if(type == Type::REVOLVE && haveSrc) {
        Group *src    = SK.GetGroup(opA);

        // Ensure the source group has loops generated (needed for OCC revolve)
        if(src->bezierLoops.l.IsEmpty()) {
            src->GenerateLoops();
        }

        double anglef = SK.GetParam(h.param(3))->val * 4; // why the 4 is needed?
        double angles = 0.0;
        if(subtype != Subtype::ONE_SIDED) {
            anglef *= 0.5;
            angles = -anglef;
        }
        Vector pt   = SK.GetEntity(predef.origin)->PointGetNum(),
               axis = SK.GetEntity(predef.entityB)->VectorGetNum();
        axis        = axis.WithMagnitude(1);

#ifdef HAVE_OPENCASCADE
        // Use OCC for partial revolution
        SBezierLoopSetSet *sblss = &(src->bezierLoops);
        SBezierLoopSet *sbls;
        for(sbls = sblss->l.First(); sbls; sbls = sblss->l.NextAfter(sbls)) {
            FaceBuilder fb = FaceBuilder::FromBezierLoopSet(sbls);
            if(!fb.IsValid()) continue;

            // Create rotation axis
            gp_Ax1 occAxis(OccUtil::ToOccPoint(pt), OccUtil::ToOccDir(axis));

            try {
                double totalAngle = anglef - angles;

                // MakeRevol only sweeps forward from where the profile sits and
                // takes no starting angle, so turn the profile back to angles.
                TopoDS_Shape profile = fb.GetFaces();
                if(angles != 0.0) {
                    gp_Trsf toStart;
                    toStart.SetRotation(occAxis, angles);
                    profile = BRepBuilderAPI_Transform(profile, toStart,
                                                       Standard_True).Shape();
                }

                // A full turn has to go through the constructor that takes no
                // angle. Asked for exactly 2*pi the other one leaves the start
                // and end of the sweep as two coincident faces, and a boolean
                // against that seam comes out invalid. The shell path draws the
                // same distinction, MakeFromRevolutionOf against
                // MakeFromHelicalRevolutionOf.
                bool fullTurn = fabs(totalAngle) >= 2 * PI - LENGTH_EPS;
                BRepPrimAPI_MakeRevol revol = fullTurn
                    ? BRepPrimAPI_MakeRevol(profile, occAxis)
                    : BRepPrimAPI_MakeRevol(profile, occAxis, totalAngle);
                if(revol.IsDone()) {
                    if(thisSolidModel->shape.IsNull()) {
                        thisSolidModel->shape = revol.Shape();
                    } else {
                        thisSolidModel->shape = BRepAlgoAPI_Fuse(
                            thisSolidModel->shape, revol.Shape()).Shape();
                    }
                }
            } catch(const Standard_Failure &e) {
                OccFailed("OCC revolve failed: %s", e.GetMessageString());
            }
        }
#else
        double dists = 0, distf = 0;
        SBezierLoopSetSet *sblss = &(src->bezierLoops);
        SBezierLoopSet *sbls;
        for(sbls = sblss->l.First(); sbls; sbls = sblss->l.NextAfter(sbls)) {
            if(fabs(anglef - angles) < 2 * PI) {
                thisShell.MakeFromHelicalRevolutionOf(sbls, pt, axis, color, this,
                                                      angles, anglef, dists, distf);
            } else {
                thisShell.MakeFromRevolutionOf(sbls, pt, axis, color, this);
            }
        }
#endif
    } else if(type == Type::HELIX && haveSrc) {
        Group *src    = SK.GetGroup(opA);
        double anglef = SK.GetParam(h.param(3))->val * 4; // why the 4 is needed?
        double dists = 0, distf = 0;
        double angles = 0.0;
        distf = SK.GetParam(h.param(7))->val * 2; // dist is applied twice
        if(subtype != Subtype::ONE_SIDED) {
            anglef *= 0.5;
            angles = -anglef;
            distf *= 0.5;
            dists = -distf;
        }
        Vector pt   = SK.GetEntity(predef.origin)->PointGetNum(),
               axis = SK.GetEntity(predef.entityB)->VectorGetNum();
        axis        = axis.WithMagnitude(1);

        SBezierLoopSetSet *sblss = &(src->bezierLoops);
        SBezierLoopSet *sbls;
        for(sbls = sblss->l.First(); sbls; sbls = sblss->l.NextAfter(sbls)) {
            thisShell.MakeFromHelicalRevolutionOf(sbls, pt, axis, color, this,
                                                  angles, anglef, dists, distf);
        }
    } else if(type == Type::LINKED) {
        // The imported shell or mesh are copied over, with the appropriate
        // transformation applied. We also must remap the face entities.
        Vector offset = {
            SK.GetParam(h.param(0))->val,
            SK.GetParam(h.param(1))->val,
            SK.GetParam(h.param(2))->val };
        Quaternion q = {
            SK.GetParam(h.param(3))->val,
            SK.GetParam(h.param(4))->val,
            SK.GetParam(h.param(5))->val,
            SK.GetParam(h.param(6))->val };

        thisMesh.MakeFromTransformationOf(&impMesh, offset, q, scale);
        thisMesh.RemapFaces(this, 0);

        thisShell.MakeFromTransformationOf(&impShell, offset, q, scale);
        thisShell.RemapFaces(this, 0);

    }
#ifdef HAVE_OPENCASCADE
    else if(type == Type::IMPORT_SOLID) {
        if(GenerateImportedSolid(srcg)) return;
    }
#endif
#ifdef HAVE_OPENCASCADE
    else if(type == Type::FILLET) {
        GenerateFillet();
    } else if(type == Type::CHAMFER) {
        GenerateChamfer();
    } else if(type == Type::SHELL) {
        GenerateShell();
    }

    if(type == Type::LOFT && opA.v != 0 && opB.v != 0) {
        GenerateLoft();
    }

    if(type == Type::SWEEP && opA.v != 0 && opB.v != 0) {
        GenerateSweep();
    }
#endif

    if(srcg->meshCombine != CombineAs::ASSEMBLE) {
        thisShell.MergeCoincidentSurfaces();
    }

    // So now we've got the mesh or shell for this group. Combine it with
    // the previous group's mesh or shell with the requested Boolean, and
    // we're done.

    Group *prevg = srcg->RunningMeshGroup();

    // For MIRROR, we want to combine with the source group's accumulated geometry,
    // not the group before the source. The mirrored geometry should be added to
    // what the source already has.
    if(type == Type::MIRROR) {
        prevg = SK.GetGroup(opA);
    }

    if(!IsForcedToMesh()) {
        SShell *prevs = &(prevg->runningShell);
        GenerateForBoolean<SShell>(prevs, &thisShell, &runningShell,
            srcg->meshCombine);

        if(srcg->meshCombine != CombineAs::ASSEMBLE) {
            runningShell.MergeCoincidentSurfaces();
        }

        // If the Boolean failed, then we should note that in the text screen
        // for this group.
        booleanFailed = runningShell.booleanFailed;
        if(booleanFailed != prevBooleanFailed) {
            SS.ScheduleShowTW();
        }
    } else {
        SMesh prevm, thism;
        prevm = {};
        thism = {};

        prevm.MakeFromCopyOf(&(prevg->runningMesh));
        prevg->runningShell.TriangulateInto(&prevm);

        thism.MakeFromCopyOf(&thisMesh);
        thisShell.TriangulateInto(&thism);

        SMesh outm = {};
        GenerateForBoolean<SMesh>(&prevm, &thism, &outm, srcg->meshCombine);

        // Remove degenerate triangles; if we don't, they'll get split in SnapToMesh
        // in every generated group, resulting in polynomial increase in triangle count,
        // and corresponding slowdown.
        outm.RemoveDegenerateTriangles();

        if(srcg->meshCombine != CombineAs::ASSEMBLE) {
            // And make sure that the output mesh is vertex-to-vertex.
            SKdNode *root = SKdNode::From(&outm);
            root->SnapToMesh(&outm);
            root->MakeMeshInto(&runningMesh);
        } else {
            runningMesh.MakeFromCopyOf(&outm);
        }

        outm.Clear();
        thism.Clear();
        prevm.Clear();
    }

#ifdef HAVE_OPENCASCADE
    // For IMPORT_SOLID + ASSEMBLE: skip all OCC shape operations
    // Just transform and display the mesh directly
    bool skipOccShapeOps = (type == Type::IMPORT_SOLID) &&
                           (srcg->meshCombine == CombineAs::ASSEMBLE);

    // A group that contributes no shape of its own - a link, a helix, a
    // sketch, a suppressed group - has to hand the accumulated solid on
    // unchanged. Clearing it and never filling it back in is what made the
    // model vanish from that group onwards. The native side already passes
    // the previous shell through in this case.
    bool contributesOccShape = thisSolidModel && !thisSolidModel->shape.IsNull();
    if(runningSolidModel && (suppress || !contributesOccShape)) {
        Group *pg = prevg;
        while(pg) {
            if(pg->runningSolidModel && !pg->runningSolidModel->shapeAcc.IsNull()) {
                // Copy all data from previous model
                runningSolidModel->shapeAcc = pg->runningSolidModel->shapeAcc;
                // Use MakeFromCopyOf to properly deep-copy the mesh (List has shallow copy)
                runningSolidModel->displayMesh.MakeFromCopyOf(&pg->runningSolidModel->displayMesh);
                runningSolidModel->faces = pg->runningSolidModel->faces;
                runningSolidModel->edges = pg->runningSolidModel->edges;
                // Only a suppressed group stands in for the one before it and
                // needs its faces to be selectable. A group that carries the
                // solid past itself has entities of its own, and adding face
                // entities beside them collides with their handles.
                if(suppress) {
                    PROFILE_SCOPE("OccFaceEntities");
                    CreateOccFaceEntities(&SK.entity);
                }
                break;
            }
            pg = pg->PreviousGroup();
        }
    }

    // Perform OCC boolean operations to build the running solid model
    if(thisSolidModel && !thisSolidModel->shape.IsNull() && !skipOccShapeOps && !suppress) {
        // An imported solid has to be put where the user put it, whether or
        // not there is anything to combine it with. Doing this only when a
        // previous solid existed left the first imported body sitting at the
        // coordinates of its own file: its own group drew it in the right
        // place, but every group after it inherited the untransformed shape.
        if(type == Type::IMPORT_SOLID) {
            Vector offset = {
                SK.GetParam(h.param(0))->val,
                SK.GetParam(h.param(1))->val,
                SK.GetParam(h.param(2))->val };
            Quaternion q = {
                SK.GetParam(h.param(3))->val,
                SK.GetParam(h.param(4))->val,
                SK.GetParam(h.param(5))->val,
                SK.GetParam(h.param(6))->val };

            // Build OCC transformation from translation and rotation
            gp_Trsf trsf;
            gp_Quaternion occQuat(q.vx, q.vy, q.vz, q.w);
            trsf.SetRotation(occQuat);
            trsf.SetTranslationPart(gp_Vec(offset.x, offset.y, offset.z));

            // Transform the shape (only needed for boolean operations)
            BRepBuilderAPI_Transform transformer(thisSolidModel->shape, trsf, Standard_True);
            runningSolidModel->shape = transformer.Shape();
        } else {
            runningSolidModel->shape = thisSolidModel->shape;
        }

        // Fillet/chamfer/shell operations already modify the accumulated shape,
        // so they don't need boolean operations
        if(type == Type::FILLET || type == Type::CHAMFER || type == Type::SHELL) {
            runningSolidModel->shapeAcc = thisSolidModel->shape;
        } else {
            SolidModelOcc::Operation occOp;
            switch(srcg->meshCombine) {
                case CombineAs::UNION:
                    occOp = SolidModelOcc::Operation::UNION;
                    break;
                case CombineAs::DIFFERENCE:
                    occOp = SolidModelOcc::Operation::DIFFERENCE;
                    break;
                case CombineAs::INTERSECTION:
                    occOp = SolidModelOcc::Operation::INTERSECTION;
                    break;
                case CombineAs::ASSEMBLE:
                default:
                    occOp = SolidModelOcc::Operation::UNION;
                    break;
            }

            // Find the previous group with valid OCC geometry
            // (sketch groups don't have OCC shapes, so we need to walk back)
            // For MIRROR/TRANSLATE/ROTATE: also check srcg if it has OCC geometry
            SolidModelOcc *prevSolid = nullptr;

            // First, check if srcg (for step-and-repeat/mirror groups) has valid OCC geometry
            if(srcg && srcg != this && srcg->runningSolidModel &&
               !srcg->runningSolidModel->shapeAcc.IsNull()) {
                // For MIRROR, we want to combine with the source group's accumulated solid
                if(type == Type::MIRROR) {
                    prevSolid = srcg->runningSolidModel;
                }
            }

            // If no srcg solid found, walk back from prevg
            if(!prevSolid) {
                Group *pg = prevg;
                while(pg) {
                    if(pg->runningSolidModel && !pg->runningSolidModel->shapeAcc.IsNull()) {
                        prevSolid = pg->runningSolidModel;
                        break;
                    }
                    pg = pg->PreviousGroup();
                }
            }

            runningSolidModel->UpdateAccumulator(occOp, prevSolid);
        }

        // Triangulate for display and extract edges
        // For IMPORT_SOLID, transform the cached mesh instead of re-triangulating
        if(type == Type::IMPORT_SOLID && !thisSolidModel->displayMesh.l.IsEmpty()) {
            // Transform the cached mesh with the offset and quaternion
            Vector offset = {
                SK.GetParam(h.param(0))->val,
                SK.GetParam(h.param(1))->val,
                SK.GetParam(h.param(2))->val };
            Quaternion q = {
                SK.GetParam(h.param(3))->val,
                SK.GetParam(h.param(4))->val,
                SK.GetParam(h.param(5))->val,
                SK.GetParam(h.param(6))->val };

            runningSolidModel->displayMesh.MakeFromTransformationOf(
                &thisSolidModel->displayMesh, offset, q, 1.0);
            // Handles for the groups that hand this mesh on; saving drops them.
            for(STriangle &tr : runningSolidModel->displayMesh.l) {
                tr.meta.face = ImportedFaceHandle(tr.meta.face);
            }

            // Also transform the edges
            runningSolidModel->edges.clear();
            for(auto &kv : thisSolidModel->edges) {
                SolidModelOcc::EdgeInfo transformed;
                transformed.index = kv.second.index;
                for(const Vector &pt : kv.second.points) {
                    Vector tpt = q.Rotate(pt).Plus(offset);
                    transformed.points.push_back(tpt);
                }
                runningSolidModel->edges[kv.first] = transformed;
            }
        } else {
            // Extract face geometry for potential use in right-click menus
            { PROFILE_SCOPE("OccExtractFaces"); runningSolidModel->ExtractFaces(); }
            // Create FACE_OCC entities for these faces so they work with selection
            CreateOccFaceEntities(&SK.entity);
            { PROFILE_SCOPE("OccTriangulate"); runningSolidModel->Triangulate(SS.ChordTolMm()); }
            // A face the mesher gave up on belongs in the same place as a
            // failed operation: the group is not showing what was asked for.
            if(occError.empty()) occError = runningSolidModel->meshError;
            { PROFILE_SCOPE("OccExtractEdges"); runningSolidModel->ExtractEdges(); }
        }
    }
#endif

    displayDirty = true;
}

void Group::GenerateDisplayItems() {
    // This is potentially slow (since we've got to triangulate a shell, or
    // to find the emphasized edges for a mesh), so we will run it only
    // if its inputs have changed.

    if(displayDirty) {
        Group *pg = RunningMeshGroup();
        bool hasOwnGeometry = !thisMesh.IsEmpty() || !thisShell.IsEmpty();
#ifdef HAVE_OPENCASCADE
        // For extrude/lathe/revolve with OCC, we have OCC geometry
        // But also check mesh/shell as fallback (e.g., MIRROR of LINKED group)
        if(type == Type::EXTRUDE || type == Type::LATHE || type == Type::REVOLVE ||
           type == Type::FILLET || type == Type::CHAMFER || type == Type::SHELL ||
           type == Type::LOFT || type == Type::SWEEP ||
           type == Type::TRANSLATE || type == Type::ROTATE || type == Type::MIRROR) {
            hasOwnGeometry = (runningSolidModel && !runningSolidModel->IsEmpty()) ||
                             !thisMesh.IsEmpty() || !thisShell.IsEmpty();
        }
        // An import set to assemble is a reference. It is shown next to the
        // solid, not in it.
        if(type == Type::IMPORT_SOLID) {
            hasOwnGeometry = !IsReference() && thisSolidModel && !thisSolidModel->IsEmpty();
            // Suppressed, it is drawn nowhere, as a suppressed extrude is not.
            if(suppress) hasOwnGeometry = false;
        }
#endif
        if(!pg && !hasOwnGeometry && type == Type::IMPORT_SOLID) {
            displayMesh.Clear();
            displayOutlines.Clear();
        } else if(pg && !hasOwnGeometry) {
            // We don't contribute any new solid model in this group, so our
            // display items are identical to the previous group's; which means
            // that we can just display those, and stop ourselves from
            // recalculating for those every time we get a change in this group.
            //
            // Note that this can end up recursing multiple times (if multiple
            // groups that contribute no solid model exist in sequence), but
            // that's okay.
            pg->GenerateDisplayItems();

            displayMesh.Clear();
            displayMesh.MakeFromCopyOf(&(pg->displayMesh));

            displayOutlines.Clear();
            if(SS.GW.showEdges || SS.GW.showOutlines) {
                displayOutlines.MakeFromCopyOf(&pg->displayOutlines);
            }
        } else {
            // We do contribute new solid model, so we have to triangulate the
            // shell, and edge-find the mesh.

            displayMesh.Clear();

#ifdef HAVE_OPENCASCADE
            // For IMPORT_SOLID: transform and display mesh directly
            const SMesh *sourceMesh = nullptr;
            if(type == Type::IMPORT_SOLID && thisSolidModel && !thisSolidModel->IsEmpty()) {
                sourceMesh = &thisSolidModel->displayMesh;
            }

            if(type == Type::IMPORT_SOLID && sourceMesh && !sourceMesh->l.IsEmpty()) {
                // Get transformation parameters
                Vector offset = {
                    SK.GetParam(h.param(0))->val,
                    SK.GetParam(h.param(1))->val,
                    SK.GetParam(h.param(2))->val };
                Quaternion q = {
                    SK.GetParam(h.param(3))->val,
                    SK.GetParam(h.param(4))->val,
                    SK.GetParam(h.param(5))->val,
                    SK.GetParam(h.param(6))->val };

                // Full mesh rendering (not dragging - we early-returned above if dragging)
                // Check if we can use cached transformed mesh
                bool quatEqual = (fabs(cachedQuat.w - q.w) < LENGTH_EPS &&
                                  fabs(cachedQuat.vx - q.vx) < LENGTH_EPS &&
                                  fabs(cachedQuat.vy - q.vy) < LENGTH_EPS &&
                                  fabs(cachedQuat.vz - q.vz) < LENGTH_EPS);
                bool cacheValid = cachedMeshValid &&
                                  cachedOffset.Equals(offset) &&
                                  quatEqual;

                if(cacheValid) {
                    // Use cached transformed mesh - just copy with color
                    for(int i = 0; i < cachedTransformedMesh.l.n; i++) {
                        STriangle tri = cachedTransformedMesh.l[i];
                        tri.meta.color = color;
                        displayMesh.AddTriangle(&tri);
                    }
                } else {
                    // Transform mesh and cache it
                    cachedTransformedMesh.Clear();
                    for(int i = 0; i < sourceMesh->l.n; i++) {
                        STriangle tri = sourceMesh->l[i];
                        tri.a = q.Rotate(tri.a).Plus(offset);
                        tri.b = q.Rotate(tri.b).Plus(offset);
                        tri.c = q.Rotate(tri.c).Plus(offset);
                        tri.an = q.Rotate(tri.an);
                        tri.bn = q.Rotate(tri.bn);
                        tri.cn = q.Rotate(tri.cn);
                        tri.meta.face = ImportedFaceHandle(tri.meta.face);
                        cachedTransformedMesh.AddTriangle(&tri);
                        tri.meta.color = color;
                        displayMesh.AddTriangle(&tri);
                    }
                    cachedOffset = offset;
                    cachedQuat = q;
                    cachedMeshValid = true;
                }
            } else if(runningSolidModel && !runningSolidModel->IsEmpty()) {
                // Use OCC triangulated mesh if available
                // Copy OCC triangulated mesh to display mesh
                for(int i = 0; i < runningSolidModel->displayMesh.l.n; i++) {
                    STriangle tri = runningSolidModel->displayMesh.l[i];
                    // OCC's ProcessFace already handles face orientation via the 'reversed' flag,
                    // so no additional winding correction is needed here
                    tri.meta.color = color;
                    displayMesh.AddTriangle(&tri);
                }
                // A link or a helix builds a native shell of its own beside
                // the solid carried over from the previous group, and both
                // belong on screen. Empty in every other case, so free.
                runningShell.TriangulateInto(&displayMesh);
            } else {
#else
            {
#endif
                runningShell.TriangulateInto(&displayMesh);
            }

            STriangle *t;
            for(t = runningMesh.l.First(); t; t = runningMesh.l.NextAfter(t)) {
                STriangle trn = *t;
                Vector n = trn.Normal();
                trn.an = n;
                trn.bn = n;
                trn.cn = n;
                displayMesh.AddTriangle(&trn);
            }

            displayOutlines.Clear();

            if(SS.GW.showEdges || SS.GW.showOutlines) {
                SOutlineList rawOutlines = {};
#ifdef HAVE_OPENCASCADE
                // For IMPORT_SOLID: handle edges based on mode
                bool handledImportSolid = false;
                if(type == Type::IMPORT_SOLID) {
                    Vector offset = {
                        SK.GetParam(h.param(0))->val,
                        SK.GetParam(h.param(1))->val,
                        SK.GetParam(h.param(2))->val };
                    Quaternion q = {
                        SK.GetParam(h.param(3))->val,
                        SK.GetParam(h.param(4))->val,
                        SK.GetParam(h.param(5))->val,
                        SK.GetParam(h.param(6))->val };

                    bool isDragging = (SS.GW.pending.operation == GraphicsWindow::Pending::DRAGGING_POINTS);
                    Vector dummy = Vector::From(0, 0, 1);

                    // Get bounding box
                    Vector minPt = {}, maxPt = {};
                    bool hasBbox = false;
                    if(thisSolidModel && !thisSolidModel->IsEmpty()) {
                        thisSolidModel->GetBoundingBox(&minPt, &maxPt);
                        hasBbox = true;
                    }

                    if(isDragging && hasBbox) {
                        // Just show bounding box edges during drag
                        Vector corners[8] = {
                            {minPt.x, minPt.y, minPt.z},
                            {maxPt.x, minPt.y, minPt.z},
                            {maxPt.x, maxPt.y, minPt.z},
                            {minPt.x, maxPt.y, minPt.z},
                            {minPt.x, minPt.y, maxPt.z},
                            {maxPt.x, minPt.y, maxPt.z},
                            {maxPt.x, maxPt.y, maxPt.z},
                            {minPt.x, maxPt.y, maxPt.z}
                        };
                        for(int i = 0; i < 8; i++) {
                            corners[i] = q.Rotate(corners[i]).Plus(offset);
                        }
                        // 12 edges of the box
                        int edges[12][2] = {
                            {0,1},{1,2},{2,3},{3,0}, // bottom
                            {4,5},{5,6},{6,7},{7,4}, // top
                            {0,4},{1,5},{2,6},{3,7}  // verticals
                        };
                        for(int i = 0; i < 12; i++) {
                            rawOutlines.AddEdge(corners[edges[i][0]], corners[edges[i][1]], dummy, dummy);
                        }
                        handledImportSolid = true;
                    } else if(thisSolidModel && !thisSolidModel->IsEmpty()) {
                        // Use thisSolidModel edges (for non-ASSEMBLE mode)
                        SEdgeList el = {};
                        thisSolidModel->MakeEdgesInto(&el);
                        for(SEdge *e = el.l.First(); e; e = el.l.NextAfter(e)) {
                            Vector a = q.Rotate(e->a).Plus(offset);
                            Vector b = q.Rotate(e->b).Plus(offset);
                            rawOutlines.AddEdge(a, b, dummy, dummy);
                        }
                        el.Clear();
                        handledImportSolid = true;
                    }
                }

                if(!handledImportSolid)
#endif
                {
                    if(!runningMesh.l.IsEmpty()) {
                        // Triangle mesh only; no shell or emphasized edges.
                        runningMesh.MakeOutlinesInto(&rawOutlines, EdgeKind::EMPHASIZED);
                    } else {
                        displayMesh.MakeOutlinesInto(&rawOutlines, EdgeKind::SHARP);
                    }
                }

                PolylineBuilder builder;
                builder.MakeFromOutlines(rawOutlines);
                builder.GenerateOutlines(&displayOutlines);
                rawOutlines.Clear();
            }
        }

        // If we render this mesh, we need to know whether it's transparent,
        // and we'll want all transparent triangles last, to make the depth test
        // work correctly.
        displayMesh.PrecomputeTransparency();

        // Recalculate mass center if needed
        if(SS.centerOfMass.draw && SS.centerOfMass.dirty && h == SS.GW.activeGroup) {
            SS.UpdateCenterOfMass();
        }
        displayDirty = false;
    }
}

Group *Group::PreviousGroup() const {
    Group *prev = nullptr;
    for(auto const &gh : SK.groupOrder) {
        Group *g = SK.GetGroup(gh);
        if(g->h == h) {
            return prev;
        }
        prev = g;
    }
    return nullptr;
}

Group *Group::RunningMeshGroup() const {
    if(type == Type::TRANSLATE || type == Type::ROTATE || type == Type::MIRROR) {
        return SK.GetGroup(opA)->RunningMeshGroup();
    } else {
        return PreviousGroup();
    }
}

bool Group::IsMeshGroup() {
    switch(type) {
        case Group::Type::EXTRUDE:
        case Group::Type::LATHE:
        case Group::Type::REVOLVE:
        case Group::Type::HELIX:
        case Group::Type::ROTATE:
        case Group::Type::TRANSLATE:
        case Group::Type::MIRROR:
        case Group::Type::LOFT:
        case Group::Type::SWEEP:
        case Group::Type::FILLET:
        case Group::Type::CHAMFER:
        case Group::Type::SHELL:
        case Group::Type::IMPORT_SOLID:
            return true;

        default:
            return false;
    }
}

bool Group::IsUsedAsSweepPath() const {
#ifdef HAVE_OPENCASCADE
    for(const auto &g : SK.group) {
        if(g.type == Type::SWEEP && g.opA.v == h.v) {
            return true;
        }
    }
#endif
    return false;
}

#ifdef HAVE_OPENCASCADE
void Group::CreateOccFaceEntities(EntityList *el) {
    if(!runningSolidModel) return;
    occFaceEntities = true;

    // Update the entity handles in the faces map to use proper FACE_OCC entities
    for(auto &kv : runningSolidModel->faces) {
        uint32_t faceIndex = kv.first;
        SolidModelOcc::FaceInfo &info = kv.second;

        // Create FACE_OCC entity for this face
        Entity en = {};
        en.type = Entity::Type::FACE_OCC;
        en.group = h;
        en.numPoint = info.point;
        // Store normal in numNormal.v{x,y,z} - use w=0 as convention
        en.numNormal = Quaternion::From(0, info.normal.x, info.normal.y, info.normal.z);

        // Generate unique entity handle using Remap
        en.h = Remap(Entity::NO_ENTITY, REMAP_OCC_FACE_BASE + faceIndex);

        el->Add(&en);

        // Update the FaceInfo to use this entity handle
        info.entityHandle = en.h.v;
    }
}
#endif

// Find the references that this group shows: imports set to assemble, up to
// this group, visible and not suppressed. If exported is true, ignore
// visibility. This also makes their meshes current.
void Group::ReferencesShown(std::vector<Group *> *refs, bool exported) const {
#ifdef HAVE_OPENCASCADE
    for(hGroup hg : SK.groupOrder) {
        Group *g = SK.GetGroup(hg);
        if(g->IsReference() && (g->visible || exported) && !g->suppress) {
            g->UpdateReference();
            refs->push_back(g);
        }
        if(hg == h) break;
    }
#endif
}

void Group::DrawMesh(DrawMeshAs how, Canvas *canvas) {
    if(!(SS.GW.showShaded ||
         SS.GW.drawOccludedAs != GraphicsWindow::DrawOccludedAs::VISIBLE)) return;

    std::vector<const SMesh *> meshes = { &displayMesh };
    std::vector<Group *> refs;
    ReferencesShown(&refs);
    for(Group *r : refs) {
        meshes.push_back(&r->reference.mesh);
    }

    switch(how) {
        case DrawMeshAs::DEFAULT: {
            // Force the shade color to something dim to not distract from
            // the sketch.
            Canvas::Fill fillFront = {};
            if(!SS.GW.showShaded) {
                fillFront.layer = Canvas::Layer::DEPTH_ONLY;
            }
            if((type == Type::DRAWING_3D || type == Type::DRAWING_WORKPLANE)
               && SS.GW.dimSolidModel) {
                fillFront.color = Style::Color(Style::DIM_SOLID);
            }
            Canvas::hFill hcfFront = canvas->GetFill(fillFront);

            // The back faces are drawn in red; should never seem them, since we
            // draw closed shells, so that's a debugging aid.
            Canvas::hFill hcfBack = {};
            if(SS.drawBackFaces && !displayMesh.isTransparent) {
                Canvas::Fill fillBack = {};
                fillBack.layer = fillFront.layer;
                fillBack.color = RgbaColor::FromFloat(1.0f, 0.1f, 0.1f);
                hcfBack = canvas->GetFill(fillBack);
            } else {
                hcfBack = hcfFront;
            }

            // Draw the shaded solid into the depth buffer for hidden line removal,
            // and if we're actually going to display it, to the color buffer too.
            for(const SMesh *m : meshes) {
                canvas->DrawMesh(*m, hcfFront, hcfBack);
            }

            // Draw mesh edges, for debugging.
            if(SS.GW.showMesh) {
                Canvas::Stroke strokeTriangle = {};
                strokeTriangle.zIndex = 1;
                strokeTriangle.color  = RgbaColor::FromFloat(0.0f, 1.0f, 0.0f);
                strokeTriangle.width  = 1;
                strokeTriangle.unit   = Canvas::Unit::PX;
                Canvas::hStroke hcsTriangle = canvas->GetStroke(strokeTriangle);
                SEdgeList edges = {};
                for(const SMesh *m : meshes) {
                    for(const STriangle &t : m->l) {
                        edges.AddEdge(t.a, t.b);
                        edges.AddEdge(t.b, t.c);
                        edges.AddEdge(t.c, t.a);
                    }
                }
                canvas->DrawEdges(edges, hcsTriangle);
                edges.Clear();
            }
            break;
        }

        case DrawMeshAs::HOVERED: {
            Canvas::Fill fill = {};
            fill.color   = Style::Color(Style::HOVERED);
            fill.pattern = Canvas::FillPattern::CHECKERED_A;
            fill.zIndex  = 2;
            Canvas::hFill hcf = canvas->GetFill(fill);

            std::vector<uint32_t> faces;
            hEntity he = SS.GW.hover.entity;
            if(he.v != 0 && SK.GetEntity(he)->IsFace()) {
                faces.push_back(he.v);
            }
            for(const SMesh *m : meshes) {
                canvas->DrawFaces(*m, faces, hcf);
            }
            break;
        }

        case DrawMeshAs::SELECTED: {
            Canvas::Fill fill = {};
            fill.color   = Style::Color(Style::SELECTED);
            fill.pattern = Canvas::FillPattern::CHECKERED_B;
            fill.zIndex  = 1;
            Canvas::hFill hcf = canvas->GetFill(fill);

            std::vector<uint32_t> faces;
            SS.GW.GroupSelection();
            auto const &gs = SS.GW.gs;
            // See also GraphicsWindow::MakeSelected "if(c >= MAX_SELECTABLE_FACES)"
            // and GraphicsWindow::GroupSelection "if(e->IsFace())"
            for(auto &fc : gs.face) {
                faces.push_back(fc.v);
            }
            for(const SMesh *m : meshes) {
                canvas->DrawFaces(*m, faces, hcf);
            }
            break;
        }
    }
}

void Group::Draw(Canvas *canvas) {
    // Everything here gets drawn whether or not the group is hidden; we
    // can control this stuff independently, with show/hide solids, edges,
    // mesh, etc.

    GenerateDisplayItems();
    DrawMesh(DrawMeshAs::DEFAULT, canvas);

    std::vector<const SOutlineList *> outlines = { &displayOutlines };
    std::vector<Group *> refs;
    ReferencesShown(&refs);
    for(Group *r : refs) {
        outlines.push_back(&r->reference.outlines);
    }

    if(SS.GW.showEdges) {
        Canvas::Stroke strokeEdge = Style::Stroke(Style::SOLID_EDGE);
        strokeEdge.zIndex = 1;
        Canvas::hStroke hcsEdge = canvas->GetStroke(strokeEdge);

        for(const SOutlineList *ol : outlines) {
            canvas->DrawOutlines(*ol, hcsEdge,
                                 SS.GW.showOutlines
                                 ? Canvas::DrawOutlinesAs::EMPHASIZED_WITHOUT_CONTOUR
                                 : Canvas::DrawOutlinesAs::EMPHASIZED_AND_CONTOUR);
        }

        if(SS.GW.drawOccludedAs != GraphicsWindow::DrawOccludedAs::INVISIBLE) {
            Canvas::Stroke strokeHidden = Style::Stroke(Style::HIDDEN_EDGE);
            if(SS.GW.drawOccludedAs == GraphicsWindow::DrawOccludedAs::VISIBLE) {
                strokeHidden.stipplePattern = StipplePattern::CONTINUOUS;
            }
            strokeHidden.layer  = Canvas::Layer::OCCLUDED;
            Canvas::hStroke hcsHidden = canvas->GetStroke(strokeHidden);

            for(const SOutlineList *ol : outlines) {
                canvas->DrawOutlines(*ol, hcsHidden,
                                     Canvas::DrawOutlinesAs::EMPHASIZED_AND_CONTOUR);
            }
        }
    }

    if(SS.GW.showOutlines) {
        Canvas::Stroke strokeOutline = Style::Stroke(Style::OUTLINE);
        strokeOutline.zIndex = 1;
        Canvas::hStroke hcsOutline = canvas->GetStroke(strokeOutline);

        for(const SOutlineList *ol : outlines) {
            canvas->DrawOutlines(*ol, hcsOutline, Canvas::DrawOutlinesAs::CONTOUR_ONLY);
        }
    }

}

void Group::DrawPolyError(Canvas *canvas) {
    const Camera &camera = canvas->GetCamera();

    Canvas::Stroke strokeUnclosed = Style::Stroke(Style::DRAW_ERROR);
    strokeUnclosed.color = strokeUnclosed.color.WithAlpha(50);
    Canvas::hStroke hcsUnclosed = canvas->GetStroke(strokeUnclosed);

    Canvas::Stroke strokeError = Style::Stroke(Style::DRAW_ERROR);
    strokeError.layer = Canvas::Layer::FRONT;
    strokeError.width = 1.0f;
    Canvas::hStroke hcsError = canvas->GetStroke(strokeError);

    double textHeight = Style::DefaultTextHeight() / camera.scale;

    // And finally show the polygons too, and any errors if it's not possible
    // to assemble the lines into closed polygons.
    if(polyError.how == PolyError::NOT_CLOSED) {
        // Report this error only in sketch-in-workplane groups; otherwise
        // it's just a nuisance. Also skip if this group is used as a sweep path,
        // since open paths are expected for sweep operations.
        if(type == Type::DRAWING_WORKPLANE && !IsUsedAsSweepPath()) {
            canvas->DrawVectorText(_("not closed contour, or not all same style!"),
                                   textHeight,
                                   polyError.notClosedAt.b, camera.projRight, camera.projUp,
                                   hcsError);
            canvas->DrawLine(polyError.notClosedAt.a, polyError.notClosedAt.b, hcsUnclosed);
        }
    } else if(polyError.how == PolyError::NOT_COPLANAR ||
              polyError.how == PolyError::SELF_INTERSECTING ||
              polyError.how == PolyError::ZERO_LEN_EDGE) {
        // These errors occur at points, not lines
        if(type == Type::DRAWING_WORKPLANE) {
            const char *msg;
            if(polyError.how == PolyError::NOT_COPLANAR) {
                msg = _("points not all coplanar!");
            } else if(polyError.how == PolyError::SELF_INTERSECTING) {
                msg = _("contour is self-intersecting!");
            } else {
                msg = _("zero-length edge!");
            }
            canvas->DrawVectorText(msg, textHeight,
                                   polyError.errorPointAt, camera.projRight, camera.projUp,
                                   hcsError);
        }
    } else {
        // The contours will get filled in DrawFilledPaths.
    }
}

void Group::DrawFilledPaths(Canvas *canvas) {
    for(const SBezierLoopSet &sbls : bezierLoops.l) {
        if(sbls.l.IsEmpty() || sbls.l[0].l.IsEmpty())
            continue;

        // In an assembled loop, all the styles should be the same; so doesn't
        // matter which one we grab.
        const SBezier *sb = &(sbls.l[0].l[0]);
        Style *s = Style::Get({ (uint32_t)sb->auxA });

        Canvas::Fill fill = {};
        fill.zIndex = 1;
        if(s->filled) {
            // This is a filled loop, where the user specified a fill color.
            fill.color = s->fillColor;
        } else if(h == SS.GW.activeGroup && SS.checkClosedContour &&
                    polyError.how == PolyError::GOOD) {
            // If this is the active group, and we are supposed to check
            // for closed contours, and we do indeed have a closed and
            // non-intersecting contour, then fill it dimly.
            fill.color = Style::Color(Style::CONTOUR_FILL).WithAlpha(127);
        } else continue;
        Canvas::hFill hcf = canvas->GetFill(fill);

        SPolygon sp = {};
        sbls.MakePwlInto(&sp);
        canvas->DrawPolygon(sp, hcf);
        sp.Clear();
    }
}

void Group::DrawContourAreaLabels(Canvas *canvas) {
    const Camera &camera = canvas->GetCamera();
    Vector gr = camera.projRight.ScaledBy(1 / camera.scale);
    Vector gu = camera.projUp.ScaledBy(1 / camera.scale);

    for(SBezierLoopSet &sbls : bezierLoops.l) {
        if(sbls.l.IsEmpty() || sbls.l[0].l.IsEmpty())
            continue;

        Vector min = sbls.l[0].l[0].ctrl[0];
        Vector max = min;
        Vector zero = Vector::From(0.0, 0.0, 0.0);
        sbls.GetBoundingProjd(Vector::From(1.0, 0.0, 0.0), zero, &min.x, &max.x);
        sbls.GetBoundingProjd(Vector::From(0.0, 1.0, 0.0), zero, &min.y, &max.y);
        sbls.GetBoundingProjd(Vector::From(0.0, 0.0, 1.0), zero, &min.z, &max.z);

        Vector mid = min.Plus(max).ScaledBy(0.5);

        hStyle hs = { Style::CONSTRAINT };
        Canvas::Stroke stroke = Style::Stroke(hs);
        stroke.layer = Canvas::Layer::FRONT;

        std::string label = SS.MmToStringSI(fabs(sbls.SignedArea()), /*dim=*/2);
        double fontHeight = Style::TextHeight(hs);
        double textWidth  = VectorFont::Builtin()->GetWidth(fontHeight, label),
               textHeight = VectorFont::Builtin()->GetCapHeight(fontHeight);
        Vector pos = mid.Minus(gr.ScaledBy(textWidth / 2.0))
                        .Minus(gu.ScaledBy(textHeight / 2.0));
        canvas->DrawVectorText(label, fontHeight, pos, gr, gu, canvas->GetStroke(stroke));
    }
}

} // namespace SolveSpace
