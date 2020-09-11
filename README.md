# stardist

This computes a skeletal structure based on a star-based distanced distance
function.

# Requirements

cmake, a c++ toolchain, CGAL, surfer2, libpugixml-dev

# Compiling
	# clone surfer and build it
	git clone --recurse-submodules https://github.com/cgalab/surfer2/
	cd surfer2
	mkdir build-lib && cd build-lib
	cmake -DBUILD_SHARED_LIBS=on -DLIB_ONLY=1 -DCMAKE_BUILD_TYPE=Release ..
	make
	cd ..
	cd ..

	# clone stardist and build it
	git clone --recurse-submodules git@gitlab.cosy.sbg.ac.at:cg/code/stardist.git
	cd stardist

	mkdir build && cd build
	cmake \
	  -DCMAKE_BUILD_TYPE=Release \
	  -DSURF_LIBRARY_PATH=<path-to-surfer-build-tree> \
	  -DSURF_INCLUDE_PATH=<path-to-surfer-source-tree> \
	  ..
	# surfer has been compiled with
	#     cmake -DBUILD_SHARED_LIBS=on -DLIB_ONLY=1 -DCMAKE_BUILD_TYPE=Release ..
	make

# Usage
The program needs two input files.  One for the site set and one for
the shape of the sites.  Both are .ipe XML files.

The site file consists of a number of ipe "markers", i.e., simple
points.  Their color (the "stroke" attribute) defines what shape they are.

The stars file needs to define the shape for each of the colors used in
the pointset.  For each color, there needs to be a marker and a single
star-shaped polygon around it, in that color.  Note that the polygon needs
to be an IPE polygon, not just a collection of line segments.

The output again is an ipe file.


The idea is that each site propagates a wavefront in the shape of its star.
This implies, that the propagation moves in different speeds in different
directions.  We want to partition the plane into a set of regions based on
which input site reached this portion of the plane first.

There are two modes:  straight skeleton and Voronoi mode.

* In the straight skeleton regime, propagations stop as soon as they reach area
  already covered by an other site.

* In Voronoi mode, the propagation continues through the other side (though
  hidden) and a wavefront may re-appear at the other side of already covered
  pieces assuming it's sufficiently fast.

## straight skeleton mode

As already suggested by the name, straight skeleton mode uses the theory of
straight skeletons to compute our structure.  Initially, around each site, we
put its corresponding star.  The stars are scaled (all with the same factor)
such that at time zero no two stars intersect.  Then we start a weighted
straight skeleton wavefront propagation.  Each edge of our input has a weight
that corresponds to its orthogonal distances to its site location.

View the propagation as a propagation upwards where the z-axis corresponds
to time.  The input edges lie in the xy-plane at t=0.  Then the wavefront
edges move in planes through their input edge and the site (x,y) location 
at time t = -1.

For computing the weighted straight skeleton of this input we use our surfer2
library.

Once the straight skeleton has been computed, we output an ipe file of the
straight skeleton faces.  Arcs between faces that were both induced by input
edges from the same site are dotted blue, arcs between different sites are
solid blue.  Note that since each input site is star shaped, arcs between
input edges from the same site never interact and only move out from their
site radially.  The first node (interaction) of each such arc is necessarily
with arcs from a different site.


## VD mode

Just like for the straight skeleton mode just described, view the problem in
three-space.  Again, consider the input stars at time t=0 around each site,
with a point for each site at (x,y, -1).  Then for each input edge (u,v) of
site s, we construct a triangle between the rays from s to u and from s to v.
This triangle is, in theory, unbounded in the positive t direction.

Then our skeletal structure consists of the lower envelope of all these triangles.
Again, arcs between triangles from the same site are not relevant here and
are only output dotted.

We use CGAL for computing the lower envelope of triangles.  For
implementational simplicity, we bound the triangles at some fixed height since
CGAL alredy comes with all the tools to compute the lower envelope of finite
triangles.  Then we confirm that the height was sufficient for our result to be
correct, and if not, we raise it and repeat.

The propagation height was sufficient if
 * in the resulting arrangement A, the outer face only has one hole
   corresponding to the set of triangles in the arrangement,
 * this set is simply connected (i.e. no uncovered area in the middle),
 * each vertex on the convex hull has degree three (i.e. no degree >= 4
   vertices on the CH), and
 * the edges leading to the convex hull are sorted radially, i.e. there are no
   interactions in the future.

Also, the resulting ipe file has faces of the same region in the same color.

