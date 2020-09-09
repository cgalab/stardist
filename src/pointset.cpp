#include "pointset.h"

#include <sstream>

#include <CGAL/Delaunay_triangulation_2.h>

#include "BasicInput.h"
#include <SkeletonStructure.h>

static std::string
node2str(const pugi::xml_node& node) { //{{{
  std::stringstream ss;
  node.print(ss);
  return ss.str();
} //}}}

class IpeMatrix { //{{{
/** An ipe transformation matrix.
 *
 * It's initialized from a string of 6 numbers and
 * can transform any point.
 */
  private:
    bool identity;
    NT m0, m1,m2,m3,m4,m5;

    void init_from_string(const std::string &m);
  public:
    //IpeMatrix();
    //IpeMatrix(const std::string &m);
    IpeMatrix(const pugi::xml_attribute &attr);

    Point_2 operator*(const Point_2& p) const;

    static IpeMatrix from_xml_attribute(const pugi::xml_attribute& attr);
};

/*
IpeMatrix::
IpeMatrix()
  : identity(true)
{}

IpeMatrix::
IpeMatrix(const std::string &m)
  : identity(false)
{
  init_from_string(m);
}
*/

IpeMatrix::
IpeMatrix(const pugi::xml_attribute& attr)
  : identity(! attr)
{
  if (! identity) {
    init_from_string(attr.value());
  }
}

void
IpeMatrix::
init_from_string(const std::string &m) {
  std::stringstream ss(m);

  ss >> m0 >> m1 >> m2 >> m3 >> m4 >> m5;
  if (ss.fail()) {
    LOG(ERROR) << "Error: parsing transformation matrix failed: " << m;
    exit(1);
  };
}

Point_2
IpeMatrix::
operator*(const Point_2& p) const {
  return identity ? p
                  : Point_2( m0*p.x() + m2*p.y() + m4,
                             m1*p.x() + m3*p.y() + m5 );
}

//}}}

class IpeElement { //{{{
  private:
    const std::string stroke_;
  protected:
    IpeMatrix matrix_;
  public:
    IpeElement(const pugi::xml_node& node);

    const std::string& stroke() const { return stroke_; };
    const IpeMatrix& matrix() const { return matrix_; };
};

IpeElement::
IpeElement(const pugi::xml_node& node)
  : stroke_(node.attribute("stroke") ? node.attribute("stroke").value() : "")
  , matrix_(node.attribute("matrix"))
{
  if (!node.attribute("stroke")) {
    LOG(ERROR) << "Error: ipe element has no stroke attribute: " << node2str(node);
    exit(1);
  };
}
//}}}

class IpeMarker : public IpeElement { //{{{
  private:
    Point_2 pos_;
  public:
    IpeMarker(const pugi::xml_node& node);
    const Point_2& pos() const { return pos_; };
};

IpeMarker::
IpeMarker(const pugi::xml_node& node)
  : IpeElement(node)
{
  /* parse position */
  if (!node.attribute("pos")) {
    LOG(ERROR) << "Error: ipe marker has no position";
    exit(1);
  };
  std::stringstream ss;
  ss.str(node.attribute("pos").value());

  NT x,y;
  ss >> x >> y;
  if (ss.fail()) {
    LOG(ERROR) << "Error: parsing pos failed: " << node.attribute("pos").value();
    exit(1);
  };

  pos_ = matrix() * Point_2(x,y);

  // LOG(INFO) << "got " << CGAL::to_double(pos().x()) << ", " << CGAL::to_double(pos().y()) << " (" << stroke() << ")";
}
//}}}

class IpePath : public IpeElement { //{{{
  private:
    std::vector<Point_2> pts_;
  public:
    IpePath(const pugi::xml_node& node);
    const std::vector<Point_2>& pts() const { return pts_; };
};

IpePath::
IpePath(const pugi::xml_node& node)
  : IpeElement(node)
{
  std::istringstream text(node.text().get());
  bool done = false;

  std::string line;
  std::stringstream ss;
  NT x, y;
  char t;
  while (std::getline(text, line)) {
    if (line == "") continue;
    if (line == "h") {
      done = true;
      break;
    }
    ss.str(line);
    ss >> x >> y >> t;
    if (ss.fail()) {
      LOG(ERROR) << "Error: parsing path failed: at '" << ss.str() << "' of " << node2str(node);
      exit(1);
    };
    if (pts_.size() == 0) {
      if (t != 'm') {
        LOG(WARNING) << "Path does not start with 'm' marker" << node2str(node);
      }
    } else {
      if (t != 'l') {
        LOG(WARNING) << "Path does not continue with 'l' marker" << node2str(node);
      }
    }
    pts_.emplace_back(matrix() * Point_2(x,y));
  };
  if (!done) {
    LOG(WARNING) << "Path did not end with 'h' marker: " << node2str(node);
  }
}
//}}}

// SurfInput {{{
class SurfInput : public BasicInput {
  private:
    int v_ctr = 0;
  public:
    int add_vertex(const Point_2& p);
    using BasicInput::add_edge;
    using BasicInput::finalize;
};

int
SurfInput::
add_vertex(const Point_2& p) {
  BasicInput::add_vertex( Vertex(p, 2, num_vertices_()) );
  return num_vertices_()-1;
}

//}}}

// Star {{{
Star::
Star(const std::vector<Point_2> p_pts, const Point_2& center) { //{{{
  if (p_pts.size() < 3) {
    LOG(ERROR) << "Too few vertices in star.";
    exit(1);
  }

  const Vector_2 c(center - CGAL::ORIGIN);
  for (const auto &p : p_pts) {
    pts.emplace_back(p - c);
  };

  // TODO: check if star-shaped
} //}}}

NT
Star::
get_max_distance_squared() const { //{{{
  auto pit = pts.begin();
  assert (pit != pts.end());

  NT max_dist = Vector_2( pit->x(), pit->y() ).squared_length();
  for (++pit; pit != pts.end(); ++pit) {
    NT this_dist = Vector_2( pit->x(), pit->y() ).squared_length();
    max_dist = std::max(max_dist, this_dist);
  }
  return max_dist;
} //}}}

void
Star::
shrink(const NT& scale) { //{{{
  for (auto &p : pts) {
    p = Point_2(p.x() / scale, p.y() / scale);
  }
}
//}}}

void
Star::
add_to_input(SurfInput& si, const Point_2& location) const { //{{{
  Point_2 o(CGAL::ORIGIN);
  Vector_2 loc_v(location - o);

  std::vector<int> indices;
  for (const auto& p : pts) {
    int idx = si.add_vertex(p + loc_v);
    indices.push_back(idx);
  };

  unsigned prev_idx = pts.size()-1;
  for (unsigned idx = 0; idx < pts.size(); ++idx) {
    Line_2 l(pts[prev_idx], pts[idx]);
    NT distance_to_origin_sq = CGAL::squared_distance(o, l);

    si.add_edge(indices[prev_idx], indices[idx], CGAL::sqrt(distance_to_origin_sq));

    prev_idx = idx;
  }
} //}}}

//}}}

// StarSet {{{
void
StarSet::
load_from_ipe(std::istream &ins) { //{{{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load(ins);

  if (! result) {
    LOG(ERROR) << "IPE file parsed with errors, attr value: [" << doc.child("node").attribute("attr").value() << "]";
    LOG(ERROR) << "Error: " << result.description();
    exit(1);
  };

  std::unordered_map<std::string, Point_2> centers;

  for (auto node : doc.select_nodes("//page/use")) {
    IpeMarker m(node.node());
    auto [_, success ] = centers.insert( {m.stroke(), m.pos()} );
    if (!success) {
      LOG(WARNING) << "Color " << m.stroke() << " has more than one center.";
    }
  };

  for (auto node : doc.select_nodes("//page/path")) {
    IpePath p(node.node());
    auto center_it = centers.find(p.stroke());
    if (center_it == centers.end()) {
      LOG(WARNING) << "No center for polygon of color " << p.stroke();
      continue;
    }
    Point_2 center = center_it->second;
    Star star(p.pts(), center);
    auto [_, success ] = insert( {p.stroke(), star} );
    if (!success) {
      LOG(WARNING) << "Color " << p.stroke() << " has more than one star.";
    }
  }

  if (size() != centers.size()) {
    LOG(WARNING) << "Different number of centers and polygons.";
  }
}
//}}}

NT
StarSet::
get_max_distance_squared() const { //{{{
  auto star_it = begin();
  assert (star_it != end());

  NT max_dist = star_it->second.get_max_distance_squared();
  for (++star_it; star_it != end(); ++star_it) {
    NT this_dist = star_it->second.get_max_distance_squared();
    max_dist = std::max(max_dist, this_dist);
  }
  return max_dist;
}
//}}}

void
StarSet::
shrink(const NT& scale) { //{{{
  for (auto &s : *this) {
    s.second.shrink(scale);
  }
}
//}}}

//}}}

// Site {{{
Site
Site::from_ipe_element(const pugi::xml_node& node, const StarSet& stars) { //{{{
  IpeMarker m(node);

  const auto star_it = stars.find(m.stroke());
  if (star_it == stars.end()) {
    LOG(ERROR) << "No star found for marker " << node2str(node);
    exit(1);
  }

  return Site(m.pos(), star_it);
} //}}}

void
Site::
add_to_input(SurfInput& si) const { //{{{
  shape().add_to_input(si, pos());
} //}}}

//}}}

// SiteSet {{{
void
SiteSet::
load_from_ipe(std::istream &ins, const StarSet& stars) { //{{{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load(ins);

  if (! result) {
    LOG(ERROR) << "IPE file parsed with errors, attr value: [" << doc.child("node").attribute("attr").value() << "]";
    LOG(ERROR) << "Error: " << result.description();
    exit(1);
  };

  for (auto node : doc.select_nodes("//page/use")) {
    sites.emplace_back(Site::from_ipe_element(node.node(), stars));
  };
} //}}}

NT
SiteSet::
get_closest_distance_squared() const { //{{{
  /** Return the distance (squared) of the closest pair.
   *
   * We get this by constructing a delaunay triangulation
   * and then iterating over all the edges.
   * */
  using Delaunay = CGAL::Delaunay_triangulation_2<Kernel>;
  Delaunay t;
  for (const auto& s : sites) {
    t.insert(s.pos());
  }

  auto eit = t.finite_edges_begin();
  if (eit == t.finite_edges_end()) {
    LOG(ERROR) << "No edges in DT of input pointset";
    exit(1);
  }

  NT min_dist = t.segment(eit).squared_length();
  for (++eit; eit != t.finite_edges_end(); ++eit) {
    NT this_dist = t.segment(eit).squared_length();
    min_dist = std::min(min_dist, this_dist);
  }
  return min_dist;
} //}}}

SurfInput
SiteSet::
make_surf_input() const { //{{{
  SurfInput si;
  for (const auto& s : sites) {
    s.add_to_input(si);
  }
  si.finalize();
  return si;
} //}}}

//}}}

// Input {{{
Input::
Input(std::istream &stars_ipe, std::istream &sites_ipe) { //{{{
  stars.load_from_ipe(stars_ipe);
  sites.load_from_ipe(sites_ipe, stars);

  /* Make sure stars are sufficiently small such that they don't
   * intersect at time t=0
   */
  NT min_site_dist = sites.get_closest_distance_squared();
  NT max_star_size = stars.get_max_distance_squared();

  LOG(INFO) << "min site dist (squared) " << min_site_dist;
  LOG(INFO) << "max star size (squared) " << max_star_size;

  NT scale = NT::getOne();
  NT scalesq = NT::getOne();
  while (max_star_size * 4 >= min_site_dist * scalesq) {
    scale   *= 2;
    scalesq *= 4;
  }
  LOG(INFO) << "scaling down stars by a factor of " << scale;
  stars.shrink(scale);
} //}}}

void
Input::
do_sk(std::ostream &os, bool write_ipe, std::string skoffset) const { //{{{
  SkeletonStructure s( sites.make_surf_input() );

  s.initialize(0 /* restrict component */);
  s.wp.advance_to_end();

  const SkeletonDCEL &sk = s.get_skeleton();
  if (write_ipe) {
    // sk.write_ipe(os, skoffset);
    skeleton_write_ipe(os, sk, sites, skoffset);
  } else {
    sk.write_obj(os);
  }
} //}}}

// }}}

// vim:set fdm=marker:
