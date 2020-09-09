#include "stardist.h"
#include "pointset.h"

#include <SkeletonStructure.h>

static void
write_ipe_segment(std::ostream& os, const Segment_2& s, const std::string &stroke) {
    os << "  <path cap=\"1\" " << stroke << ">\n";
    os << "    " << CGAL::to_double(s.source().x()) << " " << CGAL::to_double(s.source().y()) << " m\n";
    os << "    " << CGAL::to_double(s.target().x()) << " " << CGAL::to_double(s.target().y()) << " l\n";
    os << "  </path>\n";
}

void
skeleton_write_ipe(std::ostream& os, const SkeletonDCEL& sk, const SiteSet& sites, const std::string& offset_spec) {
  const std::string STROKE_INPUT        = "stroke=\"black\" pen=\"heavier\"";
  const std::string STROKE_ARC          = "stroke=\"blue\"";;
  const std::string STROKE_ARC_INTERNAL = "stroke=\"blue\" dash=\"dotted\"";
  const std::string STROKE_OFFSET       = "stroke=\"black\" pen=\"thin\" dash=\"dotted-narrower\"";

  const int ray_length = 10;
  std::vector<SkeletonDCEL::OffsetFamily> offsets;
  for (const NT& offset_distance : sk.parse_offset_spec( offset_spec )) {
    offsets.emplace_back(sk.make_offset(offset_distance));
  }

  os << "<?xml version=\"1.0\"?>\n"
        "<!DOCTYPE ipe SYSTEM \"ipe.dtd\">\n"
        "<ipe version=\"70000\" creator=\"surfer2\">\n"
        "<info bbox=\"cropbox\" />\n"

        "<ipestyle name=\"basic\">\n"
        "  <pen name=\"heavier\" value=\"0.8\"/>\n"
        "  <color name=\"blue\" value=\"0 0 1\"/>\n"
        "  <dashstyle name=\"dotted\" value=\"[1 3] 0\"/>\n"
        "  <symbol name=\"mark/disk(sx)\" transformations=\"translations\">\n"
        "    <path fill=\"sym-stroke\">\n"
        "      0.6 0 0 0.6 0 0 e\n"
        "    </path>\n"
        "  </symbol>\n"
        "</ipestyle>\n"
        "<ipestyle name=\"dashstyles\">\n"
        "  <dashstyle name=\"dotted-narrower\" value=\"[0.5 0.5] 0\"/>\n"
        "</ipestyle>\n"
        "<ipestyle name=\"surfer\">\n"
        "  <pen name=\"thin\" value=\"0.2\"/>\n"
        "</ipestyle>\n"

        "<page>\n"
        "<layer name=\"input\" />\n"
        "<layer name=\"sk0\" />\n"
        "<layer name=\"sk1\" />\n";

  if (offsets.size() > 0) {
    os << "<layer name=\"offsets\" />\n";
    os << "<group layer=\"offsets\">\n";
    for (const auto& family : offsets) {
      os << "<group>\n";
      for (const auto& segment : family) {
        write_ipe_segment(os, segment, STROKE_OFFSET);
      }
      os << "</group>\n";
    }
    os << "</group>\n";
  };

  for (unsigned arc_selector = 0; arc_selector <= 1; ++arc_selector) {
    os << "<group layer=\"sk" << arc_selector << "\">\n";
    for (auto hit = sk.halfedges_begin(); hit != sk.halfedges_end(); ++hit) {
      if (hit > hit->opposite()) continue; /* Only paint one of every halfedge pair */
      if (hit->is_input()) continue;

      bool is_arc_from_one_site = (hit->next()->is_input() || hit->prev()->is_input());
      if (is_arc_from_one_site == !!arc_selector) continue;
      std::string stroke = arc_selector ? STROKE_ARC : STROKE_ARC_INTERNAL;

      /* Arcs between SK faces of the same input start at time zero.
       * All other arcs start later.
       */

      const auto& arc = hit->curve();
      if (arc.type() == typeid(Segment_3)) {
        const Segment_3& s = boost::get<Segment_3>(arc);
        write_ipe_segment(os, project_plane(s), stroke);
      } else {
        assert(arc.type() == typeid(Ray_3));
        const Ray_3& r = boost::get<Ray_3>(arc);
        const Ray_2 r2 = project_plane(r);
        const Segment_2 s2(r2.source(), r2.point(ray_length));
        write_ipe_segment(os, s2, stroke);
      }
    }
    os << "</group>\n";
  };

  os << "<group layer=\"input\">\n";
  for (auto hit = sk.halfedges_begin(); hit != sk.halfedges_end(); ++hit) {
    if (hit > hit->opposite()) continue; /* Only paint one of every halfedge pair */
    if (!hit->is_input()) continue;

    const auto& arc = hit->curve();
    assert(arc.type() == typeid(Segment_3));
    const Segment_3& s = boost::get<Segment_3>(arc);
    write_ipe_segment(os, project_plane(s), STROKE_INPUT);
  }
  for (const auto& site : sites.get_sites()) {
    os << "<use name=\"mark/disk(sx)\" pos=\""
       << CGAL::to_double(site.pos().x()) << " " << CGAL::to_double(site.pos().y())
       << "\" size=\"normal\" stroke=\"black\"/>";
  }
  os << "</group>\n";

  os << "</page>\n"
        "</ipe>\n";
}
