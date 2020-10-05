#include "pointset.h"

#include <sstream>

#include <CGAL/Delaunay_triangulation_2.h>

#include <BasicInput.h>
#include <SkeletonStructure.h>

#include "vd.h"
#include "ipe_writer.h"

static std::string
node2str(const pugi::xml_node& node) { //{{{
  std::stringstream ss;
  node.print(ss);
  return ss.str();
} //}}}

RatNT
string2RatNT(const std::string& s) { //{{{
  /** Build a rational number from string s.
   *
   * For some reason, CGAL's rational numbers can't parse decimal strings like 3.14.
   */
  // LOG(DEBUG) << "dealing with " << s;
  RatNT n;
  size_t pos = s.find('.');
  if (pos == std::string::npos) {
    n = RatNT(s);
  } else {
    n = RatNT(s.substr(0,pos));
    std::string fractional = s.substr(pos+1);
    RatNT f(fractional);
    for (unsigned i=0; i<fractional.length(); ++i) {
      f /= 10;
    };
    n += f;
  }
  // LOG(DEBUG) << " got " << n;
  return n;
} //}}}

static RatNT
ss2RatNT(std::stringstream& ss) { //{{{
  /** Get the next token from stringstream ss and build a rational number.
   */

  // LOG(DEBUG) << "parsing " << node.attribute("pos").value();
  std::string s;
  ss >> s;
  return string2RatNT(s);
} //}}}

class IpeMatrix { //{{{
/** An ipe transformation matrix.
 *
 * It's initialized from a string of 6 numbers and
 * can transform any point.
 */
  private:
    bool identity;
    RatNT m0, m1,m2,m3,m4,m5;

    void init_from_string(const std::string &m);
  public:
    //IpeMatrix();
    //IpeMatrix(const std::string &m);
    IpeMatrix(const pugi::xml_attribute &attr);

    RatPoint_2 operator*(const RatPoint_2& p) const;

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
  m0 = ss2RatNT(ss);
  m1 = ss2RatNT(ss);
  m2 = ss2RatNT(ss);
  m3 = ss2RatNT(ss);
  m4 = ss2RatNT(ss);
  m5 = ss2RatNT(ss);
  if (ss.fail()) {
    LOG(ERROR) << "Error: parsing transformation matrix failed: " << m;
    exit(1);
  };
}

RatPoint_2
IpeMatrix::
operator*(const RatPoint_2& p) const {
  return identity ? p
                  : RatPoint_2( m0*p.x() + m2*p.y() + m4,
                                m1*p.x() + m3*p.y() + m5 );
}

//}}}

class IpeElement { //{{{
  private:
    const std::string _stroke;
  protected:
    IpeMatrix _matrix;
  public:
    IpeElement(const pugi::xml_node& node);

    const std::string& stroke() const { return _stroke; };
    const IpeMatrix& matrix() const { return _matrix; };
};

IpeElement::
IpeElement(const pugi::xml_node& node)
  : _stroke(node.attribute("stroke") ? node.attribute("stroke").value() : "")
  , _matrix(node.attribute("matrix"))
{
  if (!node.attribute("stroke")) {
    LOG(ERROR) << "Error: ipe element has no stroke attribute: " << node2str(node);
    exit(1);
  };
}
//}}}

class IpeMarker : public IpeElement { //{{{
  private:
    RatPoint_2 _pos;
  public:
    IpeMarker(const pugi::xml_node& node);
    const RatPoint_2& pos() const { return _pos; };
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

  RatNT x = ss2RatNT(ss);
  RatNT y = ss2RatNT(ss);
  if (ss.fail()) {
    LOG(ERROR) << "Error: parsing pos failed: " << node.attribute("pos").value();
    exit(1);
  };

  _pos = matrix() * RatPoint_2(x,y);

  // LOG(INFO) << "got " << CGAL::to_double(pos().x()) << ", " << CGAL::to_double(pos().y()) << " (" << stroke() << ")";
}
//}}}

class IpePath : public IpeElement { //{{{
  private:
    std::vector<RatPoint_2> _pts;
  public:
    IpePath(const pugi::xml_node& node);
    const std::vector<RatPoint_2>& pts() const { return _pts; };
};

IpePath::
IpePath(const pugi::xml_node& node)
  : IpeElement(node)
{
  std::istringstream text(node.text().get());
  bool done = false;

  std::string line;
  std::stringstream ss;
  RatNT x, y;
  char t;
  while (std::getline(text, line)) {
    if (line == "") continue;
    if (line == "h") {
      done = true;
      break;
    }
    ss.str(line);
    x = ss2RatNT(ss);
    y = ss2RatNT(ss);
    ss >> t;
    if (ss.fail()) {
      LOG(ERROR) << "Error: parsing path failed: at '" << ss.str() << "' of " << node2str(node);
      exit(1);
    };
    if (_pts.size() == 0) {
      if (t != 'm') {
        LOG(WARNING) << "Path does not start with 'm' marker" << node2str(node);
      }
    } else {
      if (t != 'l') {
        LOG(WARNING) << "Path does not continue with 'l' marker" << node2str(node);
      }
    }
    _pts.emplace_back(matrix() * RatPoint_2(x,y));
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
    int add_vertex(const RatPoint_2& p);
    using BasicInput::add_edge;
    using BasicInput::finalize;
};

int
SurfInput::
add_vertex(const RatPoint_2& p) {
  BasicInput::add_vertex( Vertex(Rat2Core(p), 2, num_vertices_()) );
  return num_vertices_()-1;
}
//}}}

// Star {{{
Star::
Star(const std::vector<RatPoint_2> pts, const RatPoint_2& center, const std::string& stroke) { //{{{
  if (pts.size() < 3) {
    LOG(ERROR) << "Too few vertices in star " << stroke << ".";
    exit(1);
  }

  const RatVector_2 c(CGAL::ORIGIN, center);
  for (const auto &p : pts) {
    _pts.emplace_back(p - c);
  };

  RatVector_2 prev_dir(CGAL::ORIGIN, _pts.back());
  for (auto p : _pts) {
    RatVector_2 dir( CGAL::ORIGIN, p);

    auto o = CGAL::orientation(prev_dir, dir);
    if (o != CGAL::LEFT_TURN) {
      LOG(ERROR) << "Input shape " << stroke << " is not (strictly) star-shaped.";
      exit(1);
    }
    prev_dir = dir;
  }
} //}}}

RatNT
Star::
get_max_vertex_distance_squared() const { //{{{
  auto pit = _pts.begin();
  assert (pit != _pts.end());

  RatNT max_dist = RatVector_2( pit->x(), pit->y() ).squared_length();
  for (++pit; pit != _pts.end(); ++pit) {
    RatNT this_dist = RatVector_2( pit->x(), pit->y() ).squared_length();
    max_dist = std::max(max_dist, this_dist);
  }
  return max_dist;
} //}}}

RatNT
Star::
get_min_edge_distance_squared() const { //{{{
  RatPoint_2 o(CGAL::ORIGIN);
  assert(_pts.size() > 0);
  RatPoint_2 prev = _pts.back();

  RatNT min_dist;
  bool first = true;

  for (const auto& pnt : _pts) {
    RatLine_2 l(prev, pnt);
    RatNT this_dist = CGAL::squared_distance(o, l);

    min_dist = first ? this_dist : std::min(min_dist, this_dist);
    first = false;

    prev = pnt;
  }
  assert(!first);
  return min_dist;
} //}}}


#if 0
void
Star::
add_to_distance_set(std::set<CoreNT>& distances) const {
  RatPoint_2 o(CGAL::ORIGIN);
  RatPoint_2 prev = _pts.back();

  for (const auto& pnt : _pts) {
    RatLine_2 l(prev, pnt);
    CoreNT distance_to_origin_sq = CGAL::sqrt( CoreNT(CGAL::squared_distance(o, l)) );

    distances.insert(distance_to_origin_sq);

    prev = pnt;
  }
}
#endif

void
Star::
shrink(const RatNT& scale) { //{{{
  for (auto &p : _pts) {
    p = RatPoint_2(p.x() / scale, p.y() / scale);
  }
}
//}}}

void
Star::
add_to_input(SurfInput& si, const RatPoint_2& location) const { //{{{
  RatPoint_2 o(CGAL::ORIGIN);
  RatVector_2 loc_v(location.x(), location.y());

  std::vector<int> indices;
  for (const auto& p : _pts) {
    int idx = si.add_vertex(p + loc_v);
    indices.push_back(idx);
  };

  unsigned prev_idx = _pts.size()-1;
  for (unsigned idx = 0; idx < _pts.size(); ++idx) {
    RatLine_2 l(_pts[prev_idx], _pts[idx]);
    RatNT distance_to_origin_sq = CGAL::squared_distance(o, l);
    si.add_edge(indices[prev_idx], indices[idx], CGAL::sqrt( CoreNT(distance_to_origin_sq) ));

    prev_idx = idx;
  }
} //}}}

void
Star::
make_vertices(std::back_insert_iterator<RatRay3List> verticesIt, const RatPoint_2& location) const { //{{{
  RatPoint_3 center(location.x(), location.y(), 0);

  for (const auto& p : _pts) {
    RatVector_3 pnt_vector(p.x(), p.y(), 1);
    verticesIt = RatRay_3( center, pnt_vector );
  };
} //}}}

void
Star::
make_triangles(std::back_insert_iterator<RealTriangleList> trianglesIt, const RatPoint_2& location, const RatNT& max_time) const { //{{{
  RatPoint_3 center(location.x(), location.y(), 0);

  std::vector<RatPoint_3> vertices;
  for (const auto& p : _pts) {
    RatVector_3 pnt_vector(p.x(), p.y(), 1);
    vertices.push_back( center + max_time * pnt_vector );
  };

  unsigned prev_idx = _pts.size()-1;
  for (unsigned idx = 0; idx < _pts.size(); ++idx) {
    trianglesIt = RatTriangle_3 (center, vertices[prev_idx], vertices[idx]);
    prev_idx = idx;
  }
} //}}}

void
Star::
add_to_input(std::back_insert_iterator<TriangleList> trianglesIt, const RatPoint_2& location, const int site_idx, const RatNT& max_time) const { //{{{
  RealTriangleList real_triangles;
  make_triangles(std::back_inserter(real_triangles), location, max_time);

  unsigned idx = 0;
  for (const auto& t : real_triangles) {
    trianglesIt = Data_triangle_3( t, {site_idx, idx} );
    ++idx;
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

  std::unordered_map<std::string, RatPoint_2> centers;

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
    RatPoint_2 center = center_it->second;
    Star star(p.pts(), center, p.stroke());
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

RatNT
StarSet::
get_max_vertex_distance_squared() const { //{{{
  auto star_it = begin();
  assert (star_it != end());

  RatNT max_dist = star_it->second.get_max_vertex_distance_squared();
  for (++star_it; star_it != end(); ++star_it) {
    RatNT this_dist = star_it->second.get_max_vertex_distance_squared();
    max_dist = std::max(max_dist, this_dist);
  }
  return max_dist;
}
//}}}

RatNT
StarSet::
get_min_edge_distance_squared() const { //{{{
  auto star_it = begin();
  assert (star_it != end());

  RatNT min_dist = star_it->second.get_min_edge_distance_squared();
  for (++star_it; star_it != end(); ++star_it) {
    RatNT this_dist = star_it->second.get_min_edge_distance_squared();
    min_dist = std::min(min_dist, this_dist);
  }
  return min_dist;
}
//}}}

#if 0
CoreNT
StarSet::
get_closest_distance() const { //{{{
  std::set<CoreNT> distances;
  distances.insert(0);
  for (auto const& star : *this) {
    star.second.add_to_distance_set(distances);
  };

  auto dist_it = distances.begin();
  assert(dist_it != distances.end());
  assert(*dist_it == 0);
  ++dist_it;
  assert(dist_it != distances.end());

  auto prev_it = dist_it;
  CoreNT closest_dist = *dist_it;
  for (++dist_it; dist_it != distances.end(); ++dist_it) {
    CoreNT this_dist = *dist_it - *prev_it;
    assert(this_dist > 0);
    closest_dist = std::min(closest_dist, this_dist);
  }

  return closest_dist;
}
//}}}
#endif

void
StarSet::
shrink(const RatNT& scale) { //{{{
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

RatNT
SiteSet::
get_closest_distance_squared() const { //{{{
  /** Return the distance (squared) of the closest pair.
   *
   * We get this by constructing a delaunay triangulation
   * and then iterating over all the edges.
   * */
  using Delaunay = CGAL::Delaunay_triangulation_2<RatKernel>;
  Delaunay t;
  for (const auto& s : sites) {
    t.insert(s.pos());
  }

  auto eit = t.finite_edges_begin();
  if (eit == t.finite_edges_end()) {
    LOG(ERROR) << "No edges in DT of input pointset";
    exit(1);
  }

  RatNT min_dist = t.segment(eit).squared_length();
  for (++eit; eit != t.finite_edges_end(); ++eit) {
    RatNT this_dist = t.segment(eit).squared_length();
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

TriangleList
SiteSet::
make_vd_input(const RatNT& max_time) const { //{{{
  TriangleList tl;
  auto it = std::back_inserter(tl);
  int idx = 0;
  for (const auto& s : sites) {
    s.add_to_input(it, idx++, max_time);
  }
  return tl;
} //}}}

RatRay3List
SiteSet::
make_vertices() const { //{{{
  RatRay3List vertices;
  auto it = std::back_inserter(vertices);
  for (const auto& s : sites) {
    s.make_vertices(it);
  }
  return vertices;
} //}}}

RealTriangleList
SiteSet::
make_triangles() const { //{{{
  RealTriangleList tl;
  auto it = std::back_inserter(tl);
  for (const auto& s : sites) {
    s.make_triangles(it, 1);
  }
  return tl;
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
  RatNT min_site_dist = sites.get_closest_distance_squared();
  RatNT max_star_size = stars.get_max_vertex_distance_squared();

  LOG(INFO) << "min site dist (squared) " << min_site_dist;
  LOG(INFO) << "max star size (squared) " << max_star_size;

  RatNT scale = 1;
  RatNT scalesq = 1;
  while (max_star_size * 4 >= min_site_dist * scalesq) {
    LOG(INFO) << "trying with scale " << scale;
    scale   *= 2;
    scalesq *= 4;
  }
  LOG(INFO) << "scaling down stars by a factor of " << scale;
  stars.shrink(scale);
} //}}}

bool
Input::
do_sk(std::ostream &os, std::string skoffset) const { //{{{
  SkeletonStructure s( sites.make_surf_input() );

  s.initialize(0 /* restrict component */);
  s.wp.advance_to_end();

  const SkeletonDCEL &sk = s.get_skeleton();
  IpeWriter().write_skeleton(os, sk, sites, skoffset);
  return true;
} //}}}

bool
Input::
do_vd(std::ostream &os, const RatNT& max_time, std::string skoffset) const { //{{{
  StarVD vd(sites, stars, max_time);

  IpeWriter().write_vd(os, vd, sites, skoffset);

  return vd.is_valid();
} //}}}

// }}}

// vim:set fdm=marker:
