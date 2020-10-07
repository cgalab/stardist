#include "pointset.h"

#include <sstream>
#include <filesystem>

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

static RatNT
string2IntegerRatNT(const std::string& s) { //{{{
  RatNT n;
  int failed = n.set_str(s.c_str());
  if (failed) {
    LOG(ERROR) << "Could not interpret string as number: " << s;
    exit(1);
  }
  return n;
} //}}}

RatNT
string2RatNT(const std::string& s) { //{{{
  /** Build a rational number from string s.
   *
   * For some reason, CGAL's rational numbers can't parse decimal strings like 3.14.
   */
  //LOG(DEBUG) << "dealing with " << s;
  RatNT n;
  size_t pos = s.find('.');
  if (pos == std::string::npos) {
    n = string2IntegerRatNT(s);
  } else {
    n = string2IntegerRatNT(s.substr(0,pos));
    std::string fractional = s.substr(pos+1);

    pos = 0;
    while (pos < s.length() && fractional[pos] == '0') {
      ++pos;
    };
    RatNT f = string2IntegerRatNT(fractional.substr(pos));
    for (unsigned i=0; i<fractional.length(); ++i) {
      f /= 10;
    };
    n += f;
  }
  if (s[0] == '-') {
    n *= -1;
  };

  //LOG(DEBUG) << std::setprecision(8) << LOG(DEBUG) << " got " << CGAL::to_double(n) << " aka " << n;
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

  verify_star(stroke);
} //}}}

Star::
Star(std::istream& ins, const std::string& fn) { //{{{
/** Load from .line file */
  unsigned numpts;
  ins >> numpts;
  if (numpts < 3) {
    LOG(ERROR) << "Too few vertices in star " << fn << ".";
    exit(1);
  }
  _pts.reserve(numpts);

  std::string x, y;
  while (ins >> x >> y) {
    RatPoint_2 p(string2RatNT(x), string2RatNT(y));
    _pts.emplace_back(p);
  }
  if (numpts != _pts.size()) {
    LOG(ERROR) << "Inconsistent number of points: expected " << numpts << " but got " << _pts.size();
    exit(1);
  }
  if (_pts.front() != _pts.back()) {
    LOG(WARNING) << "Star from line file " << fn << " is not a closed polygon.";
  } else {
    _pts.pop_back();
  }
  verify_star(fn);
} //}}}

void
Star::
verify_star(const std::string& name) { //{{{
  RatVector_2 prev_dir(CGAL::ORIGIN, _pts.back());
  int cnt = 0;
  for (auto p : _pts) {
    RatVector_2 dir( CGAL::ORIGIN, p);

    auto o = CGAL::orientation(prev_dir, dir);
    if (o != CGAL::LEFT_TURN) {
      LOG(ERROR) << "Input shape " << name << " is not (strictly) star-shaped or oriented incorrectly.";
      LOG(ERROR) << "  Indices: (" << (cnt-1) << ", " << cnt << "); ";
      LOG(ERROR) << "  pts: "
        << "(" << CGAL::to_double(prev_dir.x()) << ", " << CGAL::to_double(prev_dir.y()) << ")"
        << "; "
        << "(" << CGAL::to_double(dir     .x()) << ", " << CGAL::to_double(dir     .y()) << ")";
      exit(1);
    }
    prev_dir = dir;
    ++cnt;
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
load_from_ipe(std::istream& ins) { //{{{
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
void
StarSet::
load_lines_from_dir(const std::string& dir) { //{{{
  bool loaded_at_least_one = false;
  for (const auto& ent : std::filesystem::directory_iterator(dir)) {
    if (ent.path().extension() != ".line") {
      LOG(WARNING) << "Skipping " << ent.path() << " (We only want .line files)";
      continue;
    }

    LOG(INFO) << "Looking at " << ent.path();
    std::ifstream f(ent.path());
    if (!f.is_open()) {
      LOG(WARNING) << "Cannot open " << ent.path();
      continue;
    }

    Star star(f, ent.path());
    auto [_, success ] = insert( {ent.path().stem(), star} );
    assert(success);

    loaded_at_least_one = true;
  }

  if (! loaded_at_least_one) {
    LOG(ERROR) << "No stars loaded from " << dir;
    exit(1);
  };
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
Site::from_pos_and_shapename(const RatPoint_2& pos, const std::string& shape, const StarSet& stars) { //{{{
  const auto star_it = stars.find(shape);
  if (star_it == stars.end()) {
    LOG(ERROR) << "No star found for shape " << shape;
    exit(1);
  }

  return Site(pos, star_it);
} //}}}

Site
Site::from_ipe_element(const pugi::xml_node& node, const StarSet& stars) { //{{{
  IpeMarker m(node);
  return from_pos_and_shapename(m.pos(), m.stroke(), stars);
} //}}}

//}}}

// SiteSet {{{
Site
SiteSet::
get_one_from_line(const std::string& line, const StarSet& stars) { //{{{
  std::string x, y, shape;
  std::istringstream iss(line);

  iss >> x >> y;
  if (iss.fail()) {
    LOG(ERROR) << "Error: parsing input line: " << line;
    exit(1);
  };

  RatPoint_2 p(string2RatNT(x), string2RatNT(y));
  iss >> shape;
  if (iss.fail()) { /* Use a random shape */
    return Site(p, select_randomly(stars.begin(), stars.end()));
  } else {
    return Site::from_pos_and_shapename(p, shape, stars);
  }
} //}}}

void
SiteSet::
load_from_pnt(std::istream &ins, const StarSet& stars) { //{{{
  std::string line;

  while (std::getline(ins, line)) {
    emplace_back(get_one_from_line(line, stars));
  }
} //}}}

void
SiteSet::
load_from_line(std::istream &ins, const StarSet& stars) { //{{{
  std::string line;
  std::getline(ins, line);
  std::istringstream iss(line);

  unsigned numpts;
  iss >> numpts;
  reserve(numpts);

  while (std::getline(ins, line)) {
    Site s = get_one_from_line(line, stars);
    if ((size() > 0) && (size() == numpts - 1) && (s.pos() == front().pos())) {
      LOG(INFO) << "Not adding last point as it is the same as the first point.";
    } else {
      emplace_back(s);
    }
  }
} //}}}

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
    emplace_back(Site::from_ipe_element(node.node(), stars));
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
  for (const auto& s : *this) {
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
  for (const auto& s : *this) {
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
  for (const auto& s : *this) {
    s.add_to_input(it, idx++, max_time);
  }
  return tl;
} //}}}

RatRay3List
SiteSet::
make_vertices() const { //{{{
  RatRay3List vertices;
  auto it = std::back_inserter(vertices);
  for (const auto& s : *this) {
    s.make_vertices(it);
  }
  return vertices;
} //}}}

RealTriangleList
SiteSet::
make_triangles() const { //{{{
  RealTriangleList tl;
  auto it = std::back_inserter(tl);
  for (const auto& s : *this) {
    s.make_triangles(it, 1);
  }
  return tl;
} //}}}

//}}}

// Input {{{
Input::
Input(const std::string &stars_fn, const std::string &sites_fn, SiteFormat site_fmt, StagesPtr stages) :
  _stages(stages)
{ //{{{

  if (std::filesystem::is_directory(stars_fn)) {
    _stars.load_lines_from_dir(stars_fn);
  } else {
    std::istream *stars_ins;
    std::ifstream stars_streamin;
    if (stars_fn == "-") {
      stars_ins = &std::cin;
    } else {
      stars_streamin.open(stars_fn);
      stars_ins = &stars_streamin;
    }
    _stars.load_from_ipe(*stars_ins);
  }

  if (site_fmt == SiteFormat::guess) {
    std::filesystem::path p(sites_fn);
    if (p.extension() == ".line") site_fmt = SiteFormat::line;
    else if (p.extension() == ".pnt") site_fmt = SiteFormat::pnt;
    else site_fmt = SiteFormat::ipe;
  }

  std::istream *sites_ins;
  std::ifstream sites_streamin;
  if (sites_fn == "-") {
    sites_ins = &std::cin;
  } else {
    sites_streamin.open(sites_fn);
    sites_ins = &sites_streamin;
  }

  switch (site_fmt) {
    case SiteFormat::line:
      _sites.load_from_line(*sites_ins, _stars);
      break;
    case SiteFormat::pnt:
      _sites.load_from_pnt(*sites_ins, _stars);
      break;
    case SiteFormat::ipe:
      _sites.load_from_ipe(*sites_ins, _stars);
      break;
    case SiteFormat::guess:
    default:
      LOG(ERROR) << "Unexpected site_fmt.";
      exit(1);
  };
  _stages->push_back({"PARSING", clock()});

  preprocess();
} //}}}

void
Input::
preprocess() { //{{{
  /* Make sure stars are sufficiently small such that they don't
   * intersect at time t=0
   */
  RatNT min_site_dist = _sites.get_closest_distance_squared();
  RatNT max_star_size = _stars.get_max_vertex_distance_squared();
  _stages->push_back({"prepreprocessing", clock()});

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
  _stars.shrink(scale);
  _stages->push_back({"PREPROCESSING", clock()});
} //}}}

bool
Input::
do_sk(std::ostream &os, std::string skoffset) const { //{{{
  SkeletonStructure s( _sites.make_surf_input() );

  s.initialize(0 /* restrict component */);
  s.wp.advance_to_end();

  const SkeletonDCEL &sk = s.get_skeleton();
  IpeWriter().write_skeleton(os, sk, _sites, skoffset);
  return true;
} //}}}

bool
Input::
do_vd(std::ostream &os, const RatNT& max_time, std::string skoffset) const { //{{{
  StarVD vd(_sites, _stars, max_time, _stages);

  IpeWriter().write_vd(os, vd, _sites, skoffset);
  _stages->push_back( { "output", clock() } );

  return vd.is_valid();
} //}}}

// }}}

// vim:set fdm=marker:
