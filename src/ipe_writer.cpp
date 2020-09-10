
#include "ipe_writer.h"

#include "pointset.h"
#include "vd.h"
#include <SkeletonStructure.h>

const std::string IpeWriter::STROKE_INPUT        = "stroke=\"black\" pen=\"heavier\"";
const std::string IpeWriter::STROKE_ARC          = "stroke=\"blue\"";
const std::string IpeWriter::STROKE_ARC_INTERNAL = "stroke=\"blue\" dash=\"dotted\"";
const std::string IpeWriter::STROKE_OFFSET       = "stroke=\"black\" pen=\"thin\" dash=\"dotted-narrower\"";
const std::string IpeWriter::STROKE_UNKNOWN      = "stroke=\"red\" pen=\"heavier\"";

void
IpeWriter::
write_segment(std::ostream& os, const Segment_2& s, const std::string &stroke) { //{{{
    os << "  <path cap=\"1\" " << stroke << ">\n";
    os << "    " << CGAL::to_double(s.source().x()) << " " << CGAL::to_double(s.source().y()) << " m\n";
    os << "    " << CGAL::to_double(s.target().x()) << " " << CGAL::to_double(s.target().y()) << " l\n";
    os << "  </path>\n";
} //}}}

void
IpeWriter::
write_polygon(std::ostream& os, const std::vector<Point_2>& pts, const std::string &stroke) { //{{{
    bool first = true;
    os << "  <path cap=\"1\" " << stroke << ">\n";
    for (const auto& p : pts) {
      os << "    " << CGAL::to_double(p.x()) << " " << CGAL::to_double(p.y()) << " " << (first ? 'm' : 'l') << "\n";
      first = false;
    };
    os << "    h\n";
    os << "  </path>\n";
} //}}}

void
IpeWriter::
write_header(std::ostream& os) { //{{{
  os << "<?xml version=\"1.0\"?>\n"
        "<!DOCTYPE ipe SYSTEM \"ipe.dtd\">\n"
        "<ipe version=\"70000\" creator=\"surfer2\">\n"
        "<info bbox=\"cropbox\" />\n"

        "<ipestyle name=\"basic\">\n"
        "  <pen name=\"heavier\" value=\"0.8\"/>\n"
        "  <color name=\"blue\" value=\"0 0 1\"/>\n"
        "  <color name=\"red\" value=\"1 0 0\"/>\n"
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
        "<layer name=\"input\" />\n";
} //}}}
void
IpeWriter::
write_footer(std::ostream& os) { //{{{
  os << "</page>\n"
        "</ipe>\n";
} //}}}

void
IpeWriter::
write_sites(std::ostream& os, const SiteSet& sites) { //{{{
  os << "<group layer=\"input\">\n";
  for (const auto& site : sites.get_sites()) {
    os << "<use name=\"mark/disk(sx)\" pos=\""
       << CGAL::to_double(site.pos().x()) << " " << CGAL::to_double(site.pos().y())
       << "\" size=\"normal\" stroke=\"black\"/>";

    const Star& shape = site.shape();
    Vector_2 site_pos(site.pos().x(), site.pos().y());

    std::vector<Point_2> vertices;
    for (const auto& p : shape.pts()) {
      vertices.push_back(p + site_pos);
    };
    write_polygon(os, vertices, STROKE_INPUT);
  }
  os << "</group>\n";
} //}}}

void
IpeWriter::
write_skeleton(std::ostream& os, const SkeletonDCEL& sk, const SiteSet& sites, const std::string& offset_spec) { //{{{
  const int ray_length = 10;
  std::vector<SkeletonDCEL::OffsetFamily> offsets;
  for (const NT& offset_distance : sk.parse_offset_spec( offset_spec )) {
    offsets.emplace_back(sk.make_offset(offset_distance));
  }
  write_header(os);

  os << "<layer name=\"sk0\" />\n"
        "<layer name=\"sk1\" />\n";

  if (offsets.size() > 0) {
    os << "<layer name=\"offsets\" />\n";
    os << "<group layer=\"offsets\">\n";
    for (const auto& family : offsets) {
      os << "<group>\n";
      for (const auto& segment : family) {
        write_segment(os, segment, STROKE_OFFSET);
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
        write_segment(os, project_plane(s), stroke);
      } else {
        assert(arc.type() == typeid(Ray_3));
        const Ray_3& r = boost::get<Ray_3>(arc);
        const Ray_2 r2 = project_plane(r);
        const Segment_2 s2(r2.source(), r2.point(ray_length));
        write_segment(os, s2, stroke);
      }
    }
    os << "</group>\n";
  };

  write_sites(os, sites);
  write_footer(os);
} //}}}


void
IpeWriter::
write_vd(std::ostream& os, const StarVD& vd, const SiteSet& sites, const std::string& offset_spec) { //{{{
  const Envelope_diagram_2& diag = vd.arr();

  write_header(os);
  os << "<layer name=\"vdr\" />\n"
        "<layer name=\"vd0\" />\n"
        "<layer name=\"vd1\" />\n";

  std::vector<Segment_2> arcs[2];
  os << "<group layer=\"vdr\">\n";
  for (auto eit = diag.edges_begin(); eit != diag.edges_end(); ++eit) {
    const Point_2& p1 = eit->source()->point();
    const Point_2& p2 = eit->target()->point();
    const Segment_2 s(p1, p2);

    if (eit->number_of_surfaces() == 1) {
      write_segment(os, s, STROKE_UNKNOWN);
    } else if (eit->number_of_surfaces() == 2) {
      auto surfaces = eit->surfaces_begin();

      int site_idx_1 = surfaces->data().first;
      ++surfaces;
      int site_idx_2 = surfaces->data().first;
      ++surfaces;
      assert(surfaces == eit->surfaces_end());

      arcs[site_idx_1 == site_idx_2].push_back(s);
    } else {
      LOG(ERROR) << "Unexpected number of incident surfaces";
      exit(1);
    }
  }
  os << "</group>\n";

  for (unsigned i=0; i<=1; ++i) {
    os << "<group layer=\"vd" << i << "\">\n";
    for (auto s : arcs[i]) {
      write_segment(os, s, i ? STROKE_ARC_INTERNAL : STROKE_ARC);
    }
    os << "</group>\n";
  }

  write_sites(os, sites);
  write_footer(os);
}

// vim:set fdm=marker:
