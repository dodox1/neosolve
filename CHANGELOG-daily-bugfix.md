Changelog, daily-bugfix
=======================

The version of neosolve I run for my own work. When something gets in my way I
fix it, build a package from this branch, and carry on.

The operations this fork adds are the reason I use it: fillet, chamfer, shell,
loft, sweep and STEP are what make SolveSpace something I can build parts with,
and the native kernel cannot do them. Most of the work below is sewing up the
seam between the two kernels, and where a fix belongs upstream in SolveSpace it
goes there instead.

`CHANGELOG.md` beside this file is upstream's and ends where the fork diverged.

Where it stands
---------------

Modelling and display are in good order. Across the 190 sketches I model with,
every one loads, regenerates and renders, and the ones that still come out
invalid are invalid in upstream SolveSpace too.

STEP export works for OpenCASCADE models, which it did not before, and so do the
mesh and 2d exports. Section export is the weak one, see `Known limits`, and so
is keeping a fillet on the same edges after you edit an earlier sketch.

Unreleased
----------

### Solid modelling

* STEP export works for anything built with OpenCASCADE, which is most models,
  and gives cleaner files than the native writer: a cone comes out as one
  conical surface rather than ten spline patches.
* A link or a helix after an extrude keeps the solid instead of deleting it from
  that group onwards. Ten millimetres of a real part had been missing from every
  export of it.
* A fillet or chamfer keeps the edges you chose when the file is reopened. They
  are saved now.
* A profile inside a profile comes out as a hole.
* A revolve of a full turn no longer leaves a seam, so booleans against it work.
* A two-sided extrude or revolve is built half each way around its sketch.
* A loft uses every contour of both profiles and keeps their holes.
* A fillet can be asked for by selecting a face, which is also the only way to
  reach a circular edge. The face is remembered as geometry, so it survives
  editing the sketch.
* Sketches are solved after undo, redo, paste, deleting a request, opening a
  file and creating a group. They were not before, so degrees of freedom froze
  and a contour you had just opened still reported itself closed.
* The chord tolerance works, and neighbouring faces of different size no longer
  tear the mesh between them.

### It now tells you when something did not work

* A fillet, chamfer or shell OpenCASCADE could not build marks the group and
  says what it tried, rather than producing a group that does nothing.
* A face the mesher gave up on is named and its boundary drawn, rather than
  leaving a hole you find at the printer.
* Asking for a fillet with something other than an edge selected reports it,
  instead of quietly rounding the whole solid.

### Display

* An OpenCASCADE solid draws its edges. It drew none at all, and the 2d exports
  lost them too.
* Rounded corners draw as one surface rather than showing every tessellation
  facet.
* The crown of a revolve shades evenly.
* A sketch with one contour inside another draws without dark wedges. The
  triangulation behind them is removed: it was wrong on any non-convex outline,
  and slower than what it replaced.
* Moving the mouse over the model no longer aborts the program.

### Files

* A file you were sent can no longer write anywhere in memory through a linked
  sketch, or hang the program with an STL claiming four billion triangles.
* A missing linked image no longer kills `solvespace-cli`. One sketch in ten
  here died that way, and all of them are fine once loading survives.

### Speed

* Extruding a patterned sketch is linear, not quadratic: a 22 by 20 grille went
  from 1 minute 10 seconds to 10 seconds.
* A laser cuts a third fewer duplicated contours.

### Housekeeping

* About says neosolve, links here rather than to solvespace.com, and names
  OpenCASCADE as its licence asks.
* CI is green on all three platforms, and Linux compiles the OpenCASCADE code,
  which no job did before.
* Regression tests for extrude with a hole, two-sided extrude and revolve,
  outlines, and the triangulator.

### Taken from SolveSpace

* The solver called good constraints incompatible once a sketch passed about
  three metres. Now about 77 km.
* Constraints the solver could not satisfy were deleted instead of reported.
* Several crashes and uninitialized reads.

Known limits
------------

* **Export 2d Section is barely usable on an OpenCASCADE solid.** It returns the
  outline of a flat face lying in the chosen plane, and nothing else. A plane
  through material writes an empty file; it just tells you now.
* **Fillet and chamfer edges are stored by position**, so editing an earlier
  group can move them or make the bevel fail. Pick them again if it does. Faces
  are safe.
* **A workplane made on a face does not rotate with it.** One built from two
  edges and a point does.
* **A linked part arrives as a frozen mesh, not as geometry.** A part built with
  fillet, chamfer, shell, loft or sweep saves no surfaces, only triangles, so in
  the assembly its curves stay as coarse as they were when that part was saved
  and do not refine with the chord tolerance. Operations you add after the link
  are unaffected and work normally.
* **A linked part does not combine with a body built here.** Difference does not
  cut and the linked part is not drawn at all; union draws both, which looks like
  it worked, but they are two overlapping bodies rather than one, so the export
  reports it as self-intersecting. Assemble is the mode that behaves as intended.
  None of them deletes your model any more, which is what they used to do.
* **Loft takes two profiles only**, pairs contours by nearest centre, has no
  ruled option and cannot loft to a point.
* **Filleting every edge is all or nothing**, and the default 1 mm ignores the
  size of the part. Try a smaller radius or fewer edges.
* **A sweep result is never checked**, so impossible geometry gives a broken
  solid rather than a message.
* **Upstream SolveSpace cannot open a file containing these operations.** Files
  using only extrude, lathe and revolve open there normally.
* **Exports are not reproducible**; the tessellation is threaded. Set
  `OMP_NUM_THREADS=1` if you need the same bytes twice.
* **The test suite fails 15 checks**, eleven of them one cause. See issue #13.
