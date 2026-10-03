Working with an imported STEP
=============================

A design note for this branch. The problem: a STEP you were sent arrives as one
opaque lump. You can see it and boolean against it, and that is all. There is
nothing to snap to, nothing to dimension against, and nine separate parts behave
as one.

Everything below was measured on `tmp/step/G17_BLUE_training.stp`, a dry fire
training pistol for 3d printing, 6.5 MB, which is a fair example of what people
actually send.

Where we are today
------------------

An `IMPORT_SOLID` group is a **live link**. The `.slvs` stores only
`Group.impFileRel`, and the file is read again on every open, 4.5 seconds for
this one. The shape is kept as a real `TopoDS_Shape`, not a mesh, so booleans
and STEP re-export work and the display mesh refines with the chord tolerance.

What the group offers to the rest of the sketch is thin:

* 21 entities from `CreateBoundingBoxEntities`: a centre point, three normals,
  eight corners and twelve edges of a box.
* one `FACE_OCC` entity per face, so faces can be clicked.

That is all. No edges, no vertices, no hole centres. The only thing you can
constrain to is a box around the part, which is why it feels like a mesh even
though it is not.

What the file holds
-------------------

```
9 solids, 921 faces, 2508 edges, 1586 vertices

faces    239 plane   240 cylinder   62 torus   32 sphere   18 cone
         330 b-spline, 329 of them degree <= 3, one 5x5

edges    820 line   585 circle   140 ellipse   18 other
         945 b-spline, 938 of them degree <= 3, seven degree 6
```

The nine solids are real parts, not fragments: a pin, the magazine, a spring,
another pin, the frame with its textured grip, the slide, a block, the magazine
release and the trigger. Only the frame is invalid, and only over **2 faces of
609**, 15 mm² of 47364. The whole model is 201 x 137 x 36 mm. Every solid meshes
at 0.1 mm with no face dropped, so printing it works today.

What a `.slvs` can hold
-----------------------

Three levels, and they are easy to confuse.

**Requests** are editable sketch geometry: `LINE_SEGMENT`, `CIRCLE`,
`ARC_OF_CIRCLE`, `CUBIC`, `DATUM_POINT`, `WORKPLANE`. Constraints and dimensions
attach to these. Curves only, no surfaces, and there is no ellipse.

**A shell** is `Surface` + `SCtrl` + `TrimBy` and `Curve` + `CCtrl` + `CurvePt`
([file.cpp:413](src/file.cpp#L413)). A surface is a rational Bezier patch
`ctrl[4][4]` with weights, so degree 3 at most in each direction. Not editable;
it is what the native kernel produces.

**Triangles** are display only.

Why we are not converting to a native shell
-------------------------------------------

A rational patch of degree 3 can hold more than it first appears, piecewise: a
plane exactly, a cylinder and a cone exactly, a sphere and a torus exactly, and
any b-spline of degree <= 3 exactly once it is split at its knots. On this file
that is 920 of 921 faces.

**Measured, and it settles the question: those 921 faces become 7220 Bezier
patches.** Only 19 faces stay a single patch; the worst three give 364, 252 and
204 each. A normal SolveSpace model has tens to low hundreds of surfaces. On top
of that, every split means cutting the trim loops that cross the split and
inventing the seam curves between patches, which is the real work, and the
result has to satisfy the invariants the native booleans assume.

So the converter is realistic for a bracket and hopeless for this part, which is
exactly the kind of part you would want it for. The route is closed.

FreeCAD, for reference
----------------------

Two things they do differently, both worth knowing.

**The shape lives in the document.** `PropertyPartShape::SaveDocFile` writes a
`.brp` inside the `.FCStd` archive, so the original STEP is not needed again.
They pay for it in file size. Our link is the unusual choice, not theirs.

**Referencing is live.** The sketcher keeps `ExternalGeometry` as a reference and
`rebuildExternalGeometry()` runs on every recompute, so a sketch constrained to
an imported edge follows the solid.

What neither they nor anyone else does is turn a STEP into editable parametric
features. That is feature recognition and it is a research problem.

Note their reference is a subelement name, `"Edge12"`, which is positional. That
is their topological naming problem, lived with for fifteen years. Our own
`filletFaces` already does better: it stores a point and a normal and finds the
face geometrically, so it survives renumbering.

The plan
--------

### Stage 1, analyse the file and choose bodies

A screen listing the solids with volume, face count, size and validity, with a
checkbox each and a choice between one group per body and all in one. On confirm
it creates the groups.

For this file it would read:

```
Import solid: G17_BLUE_training.stp
9 bodies

  #  take  name        volume    faces   state
  1   [x]  solid 1       2534        5   ok
  2   [x]  solid 2      82316       70   ok
  ...
  5   [x]  solid 5     109758      609   2 bad faces
  ...

  (o) a group per body    ( ) all in one group
```

`Screen::PASTE_TRANSFORMED` and `Screen::STEP_DIMENSION` are operation screens of
the same shape, and `confscreen.cpp` already draws checkbox rows, so the
machinery exists.

Splitting pays off beyond convenience: today one invalid lump makes the whole
import invalid and every boolean against it suspect. Per body, that is confined
to the frame.

The name column needs `STEPCAFControl_Reader` instead of `STEPControl_Reader`,
which reads into an XCAF document and carries the assembly tree, the part names
and the colours. This file has no tree, so it would show `solid 1` to `solid 9`,
but a file that has one would show real names. It is not a one line swap: a
document comes back instead of a shape, `ImportSTEP` has to be rewritten and the
resulting compound may nest differently, so it needs testing against every STEP
we have.

### Stage 2, geometry from the solid into a sketch

A command over a selected face or edge that creates requests in the active group:
`LINE_SEGMENT`, `CIRCLE`, `ARC_OF_CIRCLE`, and `CUBIC` for ellipses and splines.
The same code shape as `ImportDxf`, which already calls `SS.GW.AddRequest` for
exactly these types, fed from a `TopExp_Explorer` instead of a DXF parser.

This is what makes a foreign part usable, and the numbers are why it works where
the shell conversion does not. On the frame, 138 of 609 faces are planar, and a
planar face carries 4 to 6 edges in the common case:

```
49 faces have 4 edges   23 have 5   14 have 6   5 have 7
the rest 8 to 17, one outlier with 46
```

Of the 930 edges on those planar faces, 598 are lines and 75 are circles, so 72%
convert natively. Two orders of magnitude less than 7220.

**The entities are frozen, not live.** FreeCAD needs live references because its
solids are parametric and change. An imported STEP does not change; it is a dead
file from somebody else. Frozen means ordinary requests, saved the ordinary way,
and no format change at all.

### Stage 3, a section into a sketch

`BRepAlgoAPI_Section` with the active workplane, the resulting curves added as
requests. Measured on the frame:

```
side section (XZ)   34 edges   12 line, 12 ellipse, 10 spline
front (YZ)          46 edges   32 line, 8 ellipse, 2 circle, 4 spline
top (XY)           108 edges   59 line, 15 circle, 1 ellipse, 33 spline
```

Tens of curves, which is a sketch a person would draw by hand in an afternoon,
except exact. This also pays twice: `BRepAlgoAPI_Section` is what Export 2d
Section needs, where today there is only a stopgap that returns the outline of a
flat face lying in the plane and an empty file for anything else.

Decisions taken
---------------

**No change to the `.slvs` format in any of the three stages.** Stage 1 avoids it
by writing the chosen bodies out as separate BREP files beside the `.slvs` and
pointing an ordinary `IMPORT_SOLID` group at each, rather than teaching a group
which body of a shared file it takes. BREP because it is smaller than STEP, 8.8
MB against 15 for this file, because it is OpenCASCADE's own format so nothing is
converted, and because `ImportBREP` already exists. Stage 2 and stage 3 produce
ordinary requests, which already save.

**An ellipse becomes cubics.** `CUBIC` is a non-rational Bezier, weights all 1
([drawentity.cpp:395](src/drawentity.cpp#L395)), and an ellipse is a conic, so no
polynomial of any degree holds it exactly. Four segments give about 0.03% of the
semi-axis, eight about 0.002%, which on a 20 mm feature is 5 µm and then
fractions of a micron. Accurate enough; it simply stops being an ellipse, so
there is no major axis to dimension. At the shell level this does not arise,
`SCtrl` carries a weight.

What is not in scope
--------------------

Editing the imported solid, feature recognition, and embedding the shape inside
the `.slvs`. That last one is worth doing one day for a different reason, the
missing file problem and the 4.5 seconds on every open, and it is a format change
of its own.

Open, and worth settling before stage 1
---------------------------------------

**An imported file's relative path is resolved against the working directory, not
against the `.slvs`.** `ReloadAllLinked` skips everything that is not
`Type::LINKED` ([file.cpp:1028](src/file.cpp#L1028)), so moving the model or
opening it from elsewhere makes the import fail silently. With one file this is
an annoyance. Stage 1 turns one file into nine, so it has to be fixed first or
the wizard just multiplies the fault.

**`ClearImportCache()` is written and nothing calls it**
([solidmodel.cpp:1105](src/occ/solidmodel.cpp#L1105)). Regenerate All reloads a
linked sketch and a traced image but not an imported solid, so a STEP rewritten
during a session is not picked up. One line beside `SS.images.clear()` in
`Command::REGEN_ALL`.

**Import failures only reach the console.** `ImportSTEP` is a static method with
no group to report to, so its `dbp()` lines never become `occError`. All the user
sees is `Failed to import solid from ...` with no reason, which is how a STEP
wireframe exported by mistake produces "No shapes found" and nothing useful.
