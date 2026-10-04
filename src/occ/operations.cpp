//-----------------------------------------------------------------------------
// The OpenCASCADE operations that have no native equivalent: fillet, chamfer,
// shell, loft and sweep.
//
// Split out of groupmesh.cpp unchanged.
//-----------------------------------------------------------------------------
#include "solvespace.h"

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
#include <BRepCheck_Analyzer.hxx>

#ifdef HAVE_OPENCASCADE

namespace SolveSpace {

// OpenCASCADE can report success and still hand back a solid with broken faces.
// Keep the shape, but trace those faces the way an unmeshable one is traced, so
// the damage is on screen instead of only in the geometry.
static void CheckOccResult(const TopoDS_Shape &shape, Group *g, const char *what) {
    if(shape.IsNull()) return;
    BRepCheck_Analyzer check(shape);
    if(check.IsValid()) return;

    int bad = 0, total = 0;
    for(TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face &face = TopoDS::Face(ex.Current());
        total++;
        if(check.IsValid(face)) continue;
        bad++;
        if(!SS.exportMode) MarkFaceBoundary(face);
    }
    g->OccFailed("The %s came out with %d bad face(s) of %d, marked in the "
                 "drawing. A different size often goes through.", what, bad, total);
}

// Round the selected edges of the previous group's solid.
void Group::GenerateFillet() {
    // Fillet operation: apply rounded edges to selected edges (or all if none selected)
    Group *prev = RunningMeshGroup();
    if(prev && prev->runningSolidModel && !prev->runningSolidModel->IsEmpty()) {
        try {
            // A face stands for its edges, worked out afresh from the
            // point and normal recorded when it was picked.
            std::vector<uint32_t> selectedEdges = this->selectedEdges;
            for(const FilletFace &ff : filletFaces) {
                prev->runningSolidModel->EdgesOfFace(ff.point, ff.normal,
                                                     &selectedEdges);
            }

            BRepFilletAPI_MakeFillet fillet(prev->runningSolidModel->shapeAcc);
            double radius = (valA > 0) ? valA : 1.0;

            // Iterate through edges and build source edge list for visual selection
            TopExp_Explorer explorer(prev->runningSolidModel->shapeAcc, TopAbs_EDGE);
            std::list<TopoDS_Shape> seenEdges;
            sourceEdges.clear();
            uint32_t edgeIdx = 0;
            int addedCount = 0;

            while(explorer.More()) {
                TopoDS_Edge edge = TopoDS::Edge(explorer.Current());

                // Skip duplicate edges
                bool duplicate = false;
                for(const auto &seen : seenEdges) {
                    if(seen.IsSame(edge)) {
                        duplicate = true;
                        break;
                    }
                }

                if(!duplicate) {
                    seenEdges.push_back(edge);

                    // Extract edge geometry for visual selection
                    BRepAdaptor_Curve curve(edge);
                    gp_Pnt p1 = curve.Value(curve.FirstParameter());
                    gp_Pnt p2 = curve.Value(curve.LastParameter());
                    SourceEdge se;
                    se.a = OccUtil::FromOccPoint(p1);
                    se.b = OccUtil::FromOccPoint(p2);
                    se.index = edgeIdx;
                    sourceEdges.push_back(se);

                    // If specific edges selected, only fillet those; otherwise fillet all
                    bool addEdge = selectedEdges.empty();
                    if(!addEdge) {
                        for(uint32_t selIdx : selectedEdges) {
                            if(selIdx == edgeIdx) {
                                addEdge = true;
                                break;
                            }
                        }
                    }

                    if(addEdge) {
                        fillet.Add(radius, edge);
                        addedCount++;
                    }
                    edgeIdx++;
                }
                explorer.Next();
            }

            if(addedCount > 0) {
                fillet.Build();
                if(fillet.IsDone()) {
                    thisSolidModel->shape = fillet.Shape();
                    CheckOccResult(thisSolidModel->shape, this, "fillet");
                } else {
                    OccFailed("OCC could not round %d of %d edges at radius "
                        "%g mm. Rounding only some of a solid's edges can "
                        "fail where a rounded edge meets one that is not; "
                        "rounding all of them, or a smaller radius, may "
                        "work.", addedCount, edgeIdx, radius);
                }
            }
        } catch(const Standard_Failure &e) {
            OccFailed("OCC fillet failed: %s", e.GetMessageString());
        }
    }
}

// Bevel the selected edges of the previous group's solid.
void Group::GenerateChamfer() {
    // Chamfer operation: apply beveled edges to selected edges (or all if none selected)
    Group *prev = RunningMeshGroup();
    if(prev && prev->runningSolidModel && !prev->runningSolidModel->IsEmpty()) {
        try {
            std::vector<uint32_t> selectedEdges = this->selectedEdges;
            for(const FilletFace &ff : filletFaces) {
                prev->runningSolidModel->EdgesOfFace(ff.point, ff.normal,
                                                     &selectedEdges);
            }

            BRepFilletAPI_MakeChamfer chamfer(prev->runningSolidModel->shapeAcc);
            double dist = (valA > 0) ? valA : 1.0;

            // Iterate through edges and build source edge list for visual selection
            TopExp_Explorer explorer(prev->runningSolidModel->shapeAcc, TopAbs_EDGE);
            std::list<TopoDS_Shape> seenEdges;
            sourceEdges.clear();
            uint32_t edgeIdx = 0;
            int addedCount = 0;

            while(explorer.More()) {
                TopoDS_Edge edge = TopoDS::Edge(explorer.Current());

                // Skip duplicate edges
                bool duplicate = false;
                for(const auto &seen : seenEdges) {
                    if(seen.IsSame(edge)) {
                        duplicate = true;
                        break;
                    }
                }

                if(!duplicate) {
                    seenEdges.push_back(edge);

                    // Extract edge geometry for visual selection
                    BRepAdaptor_Curve curve(edge);
                    gp_Pnt p1 = curve.Value(curve.FirstParameter());
                    gp_Pnt p2 = curve.Value(curve.LastParameter());
                    SourceEdge se;
                    se.a = OccUtil::FromOccPoint(p1);
                    se.b = OccUtil::FromOccPoint(p2);
                    se.index = edgeIdx;
                    sourceEdges.push_back(se);

                    // If specific edges selected, only chamfer those; otherwise chamfer all
                    bool addEdge = selectedEdges.empty();
                    if(!addEdge) {
                        for(uint32_t selIdx : selectedEdges) {
                            if(selIdx == edgeIdx) {
                                addEdge = true;
                                break;
                            }
                        }
                    }

                    if(addEdge) {
                        chamfer.Add(dist, edge);
                        addedCount++;
                    }
                    edgeIdx++;
                }
                explorer.Next();
            }

            if(addedCount > 0) {
                chamfer.Build();
                if(chamfer.IsDone()) {
                    thisSolidModel->shape = chamfer.Shape();
                    CheckOccResult(thisSolidModel->shape, this, "chamfer");
                } else {
                    OccFailed("OCC could not bevel %d of %d edges at %g mm. "
                        "Beveling only some of a solid's edges can fail "
                        "where a beveled edge meets one that is not; "
                        "beveling all of them, or a smaller distance, may "
                        "work.", addedCount, edgeIdx, dist);
                }
            }
        } catch(const Standard_Failure &e) {
            OccFailed("OCC chamfer failed: %s", e.GetMessageString());
        }
    }
}

// Hollow the previous group's solid out to a wall thickness.
void Group::GenerateShell() {
    // Shell operation: hollow out the solid with specified wall thickness
    Group *prev = RunningMeshGroup();
    if(prev && prev->runningSolidModel && !prev->runningSolidModel->IsEmpty()) {
        try {
            double thickness = (valA > 0) ? valA : 1.0;

            // Collect all faces with their indices
            TopTools_ListOfShape facesToRemove;
            std::vector<TopoDS_Face> allFaces;
            std::list<TopoDS_Shape> seenFaces;

            TopExp_Explorer faceExp(prev->runningSolidModel->shapeAcc, TopAbs_FACE);
            while(faceExp.More()) {
                TopoDS_Face face = TopoDS::Face(faceExp.Current());

                // Skip duplicate faces
                bool duplicate = false;
                for(const auto &seen : seenFaces) {
                    if(seen.IsSame(face)) {
                        duplicate = true;
                        break;
                    }
                }

                if(!duplicate) {
                    seenFaces.push_back(face);
                    allFaces.push_back(face);
                }
                faceExp.Next();
            }

            if(!allFaces.empty()) {
                if(!selectedFaces.empty()) {
                    // Use selected faces
                    for(uint32_t idx : selectedFaces) {
                        if(idx < allFaces.size()) {
                            facesToRemove.Append(allFaces[idx]);
                        }
                    }
                } else {
                    // Auto: find largest face
                    double maxArea = 0;
                    size_t maxIdx = 0;
                    for(size_t i = 0; i < allFaces.size(); i++) {
                        GProp_GProps props;
                        BRepGProp::SurfaceProperties(allFaces[i], props);
                        double area = props.Mass();
                        if(area > maxArea) {
                            maxArea = area;
                            maxIdx = i;
                        }
                    }
                    facesToRemove.Append(allFaces[maxIdx]);
                }
            }

            BRepOffsetAPI_MakeThickSolid shellMaker;
            shellMaker.MakeThickSolidByJoin(
                prev->runningSolidModel->shapeAcc,
                facesToRemove,
                -thickness,  // negative = inward offset
                Precision::Confusion()
            );

            if(shellMaker.IsDone()) {
                thisSolidModel->shape = shellMaker.Shape();
                CheckOccResult(thisSolidModel->shape, this, "shell");
            } else {
                OccFailed("Shell operation failed");
            }
        } catch(const Standard_Failure &e) {
            OccFailed("OCC shell failed: %s", e.GetMessageString());
        }
    }
}

// Build a solid between the contours of two profiles.
void Group::GenerateLoft() {
    Group *srcA = SK.GetGroup(opA);
    Group *srcB = SK.GetGroup(opB);

    if(srcA && srcB &&
       srcA->polyError.how == PolyError::GOOD &&
       srcB->polyError.how == PolyError::GOOD) {

        // A contour that can be lofted: its outer wire and the wire of each
        // hole in it. One sketch can hold several.
        struct LoftProfile {
            TopoDS_Wire outer;
            std::vector<TopoDS_Wire> holes;
        };

        auto ProfilesOf = [](Group *g, std::vector<LoftProfile> *profiles) {
            SBezierLoopSetSet *sblss = &(g->bezierLoops);
            for(SBezierLoopSet *sbls = sblss->l.First(); sbls;
                sbls = sblss->l.NextAfter(sbls))
            {
                FaceBuilder fb = FaceBuilder::FromBezierLoopSet(sbls);
                if(!fb.IsValid() || fb.GetOuterWire().IsNull()) continue;
                LoftProfile p;
                p.outer = fb.GetOuterWire();
                for(const TopoDS_Wire &w : fb.GetWires()) {
                    if(!w.IsSame(p.outer)) p.holes.push_back(w);
                }
                profiles->push_back(p);
            }
        };

        auto CentreOf = [](const TopoDS_Wire &w) {
            GProp_GProps props;
            BRepGProp::LinearProperties(w, props);
            return props.CentreOfMass();
        };

        // The loops of the two sketches need not come out in the same order,
        // so pair them by how close their centres are.
        auto NearestFree = [](const gp_Pnt &c, const std::vector<gp_Pnt> &cs,
                              std::vector<bool> *taken) -> size_t {
            size_t best = cs.size();
            double bestDist = 0.0;
            for(size_t j = 0; j < cs.size(); j++) {
                if((*taken)[j]) continue;
                double d = c.Distance(cs[j]);
                if(best == cs.size() || d < bestDist) {
                    best = j;
                    bestDist = d;
                }
            }
            if(best < cs.size()) (*taken)[best] = true;
            return best;
        };

        auto LoftBetween = [](const TopoDS_Wire &a, const TopoDS_Wire &b,
                              TopoDS_Shape *shape) -> bool {
            // isSolid=true makes a solid rather than a shell; ruled=false
            // makes the transition smooth rather than straight.
            BRepOffsetAPI_ThruSections builder(Standard_True, Standard_False);
            builder.AddWire(a);
            builder.AddWire(b);
            builder.Build();
            if(!builder.IsDone()) return false;
            *shape = builder.Shape();
            return true;
        };

        std::vector<LoftProfile> profA, profB;
        ProfilesOf(srcA, &profA);
        ProfilesOf(srcB, &profB);

        // ThruSections dereferences a null curve and segfaults when it is
        // built with fewer than two sections, so don't even try.
        if(profA.empty() || profB.empty()) {
            OccFailed("OCC loft: need two usable profiles");
        } else try {
            if(profA.size() != profB.size()) {
                OccFailed("OCC loft: the sketches hold %d and %d contours, so only the "
                    "%d nearest pairs are lofted",
                    (int)profA.size(), (int)profB.size(),
                    (int)std::min(profA.size(), profB.size()));
            }

            std::vector<gp_Pnt> centresB;
            for(const LoftProfile &p : profB) centresB.push_back(CentreOf(p.outer));
            std::vector<bool> takenB(profB.size(), false);

            TopoDS_Shape shape;
            bool haveShape = false;
            for(const LoftProfile &a : profA) {
                size_t bi = NearestFree(CentreOf(a.outer), centresB, &takenB);
                if(bi >= profB.size()) continue;
                const LoftProfile &b = profB[bi];

                TopoDS_Shape solid;
                if(!LoftBetween(a.outer, b.outer, &solid)) {
                    OccFailed("OCC loft: OCC could not loft between two contours");
                    continue;
                }

                // ThruSections reads every wire it is given as one more section
                // along the loft, so a hole cannot be handed to it. Loft the
                // holes into solids of their own and cut those out instead.
                if(a.holes.size() != b.holes.size()) {
                    OccFailed("OCC loft: the paired contours have %d and %d holes, so "
                        "only the %d nearest pairs of holes are lofted",
                        (int)a.holes.size(), (int)b.holes.size(),
                        (int)std::min(a.holes.size(), b.holes.size()));
                }

                std::vector<gp_Pnt> holeCentresB;
                for(const TopoDS_Wire &w : b.holes) {
                    holeCentresB.push_back(CentreOf(w));
                }
                std::vector<bool> takenHoles(b.holes.size(), false);

                for(size_t i = 0; i < a.holes.size(); i++) {
                    size_t hi = NearestFree(CentreOf(a.holes[i]), holeCentresB,
                                            &takenHoles);
                    if(hi >= b.holes.size()) continue;

                    TopoDS_Shape hole;
                    if(!LoftBetween(a.holes[i], b.holes[hi], &hole)) {
                        OccFailed("OCC loft: OCC could not loft hole %d", (int)i);
                        continue;
                    }
                    BRepAlgoAPI_Cut cut(solid, hole);
                    if(cut.IsDone()) {
                        solid = cut.Shape();
                    } else {
                        OccFailed("OCC loft: OCC could not cut hole %d", (int)i);
                    }
                }

                if(!haveShape) {
                    shape = solid;
                    haveShape = true;
                } else {
                    BRepAlgoAPI_Fuse fuse(shape, solid);
                    if(fuse.IsDone()) {
                        shape = fuse.Shape();
                    } else {
                        OccFailed("OCC loft: OCC could not fuse the lofted contours");
                    }
                }
            }

            if(haveShape) {
                thisSolidModel->shape = shape;
                CheckOccResult(thisSolidModel->shape, this, "loft");
            }
        } catch(const Standard_Failure &e) {
            OccFailed("OCC loft failed: %s", e.GetMessageString());
        }
    }
}

// Sweep a profile along a path.
void Group::GenerateSweep() {
    Group *pathGroup = SK.GetGroup(opA);      // The path to sweep along
    Group *profileGroup = SK.GetGroup(opB);  // The profile to sweep

    if(profileGroup && pathGroup) {
        try {
            // Get the path wire - try closed loops first, then open paths
            TopoDS_Wire pathWire;

            // First try closed loops
            SBezierLoopSetSet *pathLoops = &(pathGroup->bezierLoops);
            for(SBezierLoopSet *sbls = pathLoops->l.First(); sbls; sbls = pathLoops->l.NextAfter(sbls)) {
                FaceBuilder fb = FaceBuilder::FromBezierLoopSet(sbls);
                if(fb.IsValid()) {
                    pathWire = fb.GetOuterWire();
                    break;
                }
            }

            // If no closed loop, try open paths from bezierOpens
            if(pathWire.IsNull() && !pathGroup->bezierOpens.l.IsEmpty()) {
                // Get all beziers from the open path and build a wire
                SBezierList sbl = {};
                for(int i = 0; i < pathGroup->bezierOpens.l.n; i++) {
                    const SBezierLoop *loop = &pathGroup->bezierOpens.l[i];
                    for(int j = 0; j < loop->l.n; j++) {
                        sbl.l.Add(&loop->l[j]);
                    }
                }
                if(sbl.l.n > 0) {
                    pathWire = FaceBuilder::WireFromBezierList(&sbl, pathGroup->bezierOpens.normal);
                }
                sbl.l.Clear();
            }

            if(!pathWire.IsNull()) {
                // Create pipe shell builder with the path
                BRepOffsetAPI_MakePipeShell pipeBuilder(pathWire);

                // Get the profile wire - try closed loops first, then open paths
                TopoDS_Wire profileWire;

                SBezierLoopSetSet *profileLoops = &(profileGroup->bezierLoops);
                for(SBezierLoopSet *sbls = profileLoops->l.First(); sbls; sbls = profileLoops->l.NextAfter(sbls)) {
                    FaceBuilder fb = FaceBuilder::FromBezierLoopSet(sbls);
                    if(fb.IsValid()) {
                        profileWire = fb.GetOuterWire();
                        break;
                    }
                }

                // Try open profile if no closed loop found
                if(profileWire.IsNull() && !profileGroup->bezierOpens.l.IsEmpty()) {
                    SBezierList sbl = {};
                    for(int i = 0; i < profileGroup->bezierOpens.l.n; i++) {
                        const SBezierLoop *loop = &profileGroup->bezierOpens.l[i];
                        for(int j = 0; j < loop->l.n; j++) {
                            sbl.l.Add(&loop->l[j]);
                        }
                    }
                    if(sbl.l.n > 0) {
                        profileWire = FaceBuilder::WireFromBezierList(&sbl, profileGroup->bezierOpens.normal);
                    }
                    sbl.l.Clear();
                }

                if(!profileWire.IsNull()) {
                    // Add profile: WithContact=true positions profile at path start,
                    // WithCorrection=true rotates to be orthogonal to path
                    pipeBuilder.Add(profileWire, Standard_True, Standard_True);
                    pipeBuilder.Build();

                    if(pipeBuilder.IsDone()) {
                        pipeBuilder.MakeSolid();
                        thisSolidModel->shape = pipeBuilder.Shape();
                        CheckOccResult(thisSolidModel->shape, this, "sweep");
                    }
                }
            }
        } catch(const Standard_Failure &e) {
            OccFailed("OCC sweep failed: %s", e.GetMessageString());
        }
    }
}

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
