
#include "vd.h"

#include "pointset.h"

bool
StarVD::
check_sufficiently_far() { //{{{
  /** Check if we have made our triangles sufficiently large for the VD to be correct.
   */

  using Face_handle                             = typename Envelope_diagram_2::Face_handle;
  using Vertex_const_handle                     = typename Envelope_diagram_2::Vertex_const_handle;
  using Halfedge_const_handle                   = typename Envelope_diagram_2::Halfedge_const_handle;
  using Inner_ccb_iterator                      = typename Envelope_diagram_2::Inner_ccb_iterator;
  using Ccb_halfedge_const_circulator           = typename Envelope_diagram_2::Ccb_halfedge_const_circulator;
  using Halfedge_around_vertex_const_circulator = typename Envelope_diagram_2::Halfedge_around_vertex_const_circulator;
  using Halfedge_around_vertex_const_circulator = typename Envelope_diagram_2::Halfedge_around_vertex_const_circulator;

  _new_max_time = _max_time;

  /* If the unbounded face has more than one hole, then our triangles did not go up far enough to meet.
   * (The reverse does not hold.  There could be area not reached by the wavefront
   *  somewhere in the middle, or not all events on the outer face have been seen yet.)
   */
  const Face_handle& unbounded_face = _arr.fictitious_face();
  assert(unbounded_face->is_unbounded());
  if (unbounded_face->number_of_holes() != 1) {
    LOG(WARNING) << "Unbounded face has " << unbounded_face->number_of_holes() << " holes.  "
                 << "This indicates that the propagation did not process sufficiently far.";
    _new_max_time *= 2;
    return false;
  };

  /* Next, check if there is only one connected set of surfaces
   * in the envelope, and that set is simply connected.
   *
   * We do this by iterating over all edges, and if an
   * edge only has one incident "surface", check if the edge is
   * incident to the unbounded face.
   */
  for (auto eit = _arr.edges_begin(); eit != _arr.edges_end(); ++eit) {
    if (eit->number_of_surfaces() == 2) {
      continue;
    } else if (eit->number_of_surfaces() != 1) {
      LOG(ERROR) << "Unexpected number of incident surfaces";
      exit(1);
    };
    if (! eit->face()->is_unbounded() &&
        ! eit->twin()->face()->is_unbounded()) {
      LOG(WARNING) << "Found boundary edge not incident to the unbounded face.  "
                   << "This indicates the propagation left a single, multiply-connected set of surfaces in the lower envelope.";
      _new_max_time *= 2;
      return false;
    }
  }

  /* Now that there is only one, simply-connected set of surfaces,
   * iterate along the convex hull to check if all ray-arcs are pointing away from each
   * other, and thus, no more interactions could happen.
   */
  std::vector<Halfedge_const_handle> rays;

  Inner_ccb_iterator holes_it = unbounded_face->holes_begin();
  assert(holes_it != unbounded_face->holes_end());

  Ccb_halfedge_const_circulator ccb = *holes_it;
  do {
    const Vertex_const_handle& vertex = ccb->target();
    unsigned degree = vertex->degree();
    assert(degree >= 3);
    if (degree > 3) {
      LOG(WARNING) << "Found boundary vertex with degree of " << degree << " (which is > 3).  "
                   << "This might indicate the propagation has not finished.";
      _new_max_time *= 2;
      return false;
    };

    /* find edge pointing outside */
    [[maybe_unused]] bool found_interior_arc = false;
    Halfedge_around_vertex_const_circulator ec = vertex->incident_halfedges();
    for (unsigned i=0; i<degree; ++i, ++ec) {
      assert(ec->target() == vertex);
      if (! ec->face()->is_unbounded() &&
          ! ec->twin()->face()->is_unbounded()) {
        assert(!found_interior_arc);
        found_interior_arc = true;
        rays.emplace_back( ec);
      }
    }
    assert(ec == vertex->incident_halfedges());
    assert(found_interior_arc);

    ++ccb;
  } while (ccb != *holes_it);
  assert((++holes_it) == unbounded_face->holes_end());

  bool arcs_colliding = false;

  Halfedge_const_handle prev_ray = rays.back();
  Vector_2              prev_dir( prev_ray->source()->point(), prev_ray->target()->point() );
  // LOG(INFO) << "ray from " << prev_ray->source()->point()<< " to " << prev_ray->target()->point();
  for (auto ray : rays) {
    // LOG(INFO) << "ray from " << ray->source()->point() << " to " << ray->target()->point();
    Vector_2 dir( ray->source()->point(), ray->target()->point()) ;

    auto o = CGAL::orientation(prev_dir, dir);
    if (o == CGAL::LEFT_TURN) {
      arcs_colliding = true;

      /* Try to figure out when the magic happens. */
      auto sit = prev_ray->surfaces_begin(); assert( sit != prev_ray->surfaces_end() );
      const auto& s11 = *sit; sit++;          assert( sit != prev_ray->surfaces_end() );
      const auto& s12 = *sit; sit++;          assert( sit == prev_ray->surfaces_end() );

      sit = ray->surfaces_begin();  assert( sit != ray->surfaces_end() );
      const auto& s21 = *sit; sit++; assert( sit != ray->surfaces_end() );
      const auto& s22 = *sit; sit++; assert( sit == ray->surfaces_end() );

      assert( ! s11.is_segment() );
      assert( ! s12.is_segment() );
      assert( ! s21.is_segment() );
      assert( ! s22.is_segment() );

      const Plane_3& p11 = s11.plane();
      const Plane_3& p12 = s12.plane();
      const Plane_3& p21 = s21.plane();
      const Plane_3& p22 = s22.plane();

      assert( p11 != p12 );
      assert( p21 != p22 );
      // LOG(INFO) << "  incident plane 11: " << p11;
      // LOG(INFO) << "  incident plane 12: " << p12;
      // LOG(INFO) << "  incident plane 21: " << p21;
      // LOG(INFO) << "  incident plane 22: " << p22;
      // LOG(INFO) << "p11 == p21: " << (p11 == p21);
      // LOG(INFO) << "p11 == p22: " << (p11 == p22);
      // LOG(INFO) << "p12 == p21: " << (p12 == p21);
      // LOG(INFO) << "p12 == p22: " << (p12 == p22);

      assert( ((p11 == p21) + (p11 == p22) +
               (p12 == p21) + (p12 == p22)) == 1); // exactly one surface in common

      const Plane_3& p1 = p11;
      const Plane_3& p2 = p12;
      const Plane_3& p3 = ( p21 == p11 || p21 == p12 ) ? p22 : p21;

      //LOG(INFO) << "  incident plane 1: " << p1;
      //LOG(INFO) << "  incident plane 2: " << p2;
      //LOG(INFO) << "  incident plane 3: " << p3;

      CGAL::cpp11::result_of<Kernel::Intersect_3(Plane_3, Plane_3)>::type res1 = intersection(p1, p2);
      assert(res1);
      const Line_3* line = boost::get<Line_3>(&*res1);
      assert(line);

      CGAL::cpp11::result_of<Kernel::Intersect_3(Line_3, Plane_3)>::type res2 = intersection(*line, p3);
      assert(res2);
      const Point_3* pnt = boost::get<Point_3>(&*res2);
      assert(pnt);

      // LOG(INFO) << "  arcs meet in " << *pnt;
      while (_new_max_time < pnt->z()) {
        _new_max_time *= 2;
      }
    };

    prev_ray = ray;
    prev_dir = dir;
  }
  if (arcs_colliding) {
    LOG(INFO) << "Arcs are on a collision course.  "
              << "The propagation has not finished yet.  "
              << "Recommend new minimum --vd-max of " << CGAL::to_double(_new_max_time) << ".";
  };

  return true;
} //}}}

StarVD::
StarVD(const SiteSet& sites, const NT& max_time, const bool auto_height) //{{{
  : _max_time(max_time)
  , _auto_height(auto_height)
{
  while (1) {
    LOG(DEBUG) << " preparing triangles";
    _triangles = sites.make_vd_input(_max_time);
    LOG(DEBUG) << " computing lower envelope";
    CGAL::lower_envelope_3 (_triangles.begin(), _triangles.end(), _arr);

    _is_valid = check_sufficiently_far();
    if (_is_valid) {
      LOG(INFO) << "VD looks good with upper height of " << CGAL::to_double(_max_time) << ".";
      break;
    }

    LOG(WARNING) << "Triangles too small with height " << CGAL::to_double(_max_time) << ".";

    if (! _auto_height) break;
    assert(_new_max_time > _max_time);

    _max_time = _new_max_time;
    _arr.clear();
    LOG(INFO) << "  Trying again with max height of " << CGAL::to_double(_max_time) << ".";
  };
} //}}}


// vim:set fdm=marker:
