#pragma once

#include "stardist.h"

#include <unordered_map>
#include <pugixml.hpp>

RatNT string2RatNT(const std::string& s);

using RatRay3List = std::vector<RatRay_3>;

class SurfInput;

class Star {
  private:
    std::vector<RatPoint_2> _pts;

    void verify_star(const std::string& name);
  public:
    Star(const std::vector<RatPoint_2> pts, const RatPoint_2& center, const std::string& stroke /* for logging/warning purposes only */);
    Star(std::istream& ins, const std::string& fn);

    RatNT get_max_vertex_distance_squared() const;
    RatNT get_min_edge_distance_squared() const;
    // void add_to_distance_set(std::set<CoreNT>& distances) const;
    const std::vector<RatPoint_2>& pts() const { return _pts; };
    unsigned size() const { return _pts.size(); };

    void scale(const RatNT& scale);

    void add_to_input(SurfInput& si, const RatPoint_2& p) const;

    void make_vertices(std::back_insert_iterator<RatRay3List> verticesIt, const RatPoint_2& location) const;
    void make_triangles(std::back_insert_iterator<RealTriangleList> trianglesIt, const RatPoint_2& location, const RatNT& max_time) const;
    void add_to_input(std::back_insert_iterator<TriangleList> trianglesIt, const RatPoint_2& p, const int site_idx, const RatNT& max_time) const;
};

class StarSet : private std::unordered_map<std::string, Star> {
  private:
    using Base = std::unordered_map<std::string, Star>;
  public:
    using It = Base::const_iterator;

    void load_from_ipe(std::istream& ins);
    void load_stars_from_dir(const std::string& dir, StarFormat star_fmt);
    void load_stars_from_file(const std::string& fn, StarFormat star_fmt, bool accept_stdio);

    using Base::find;
    using Base::begin;
    using Base::end;

    RatNT get_max_vertex_distance_squared() const;
    RatNT get_min_edge_distance_squared() const;
    void scale(const RatNT& scale);
    void scale_random(double random_scale_sigma);
};

class Site {
  private:
    const RatPoint_2 _pos;
    const StarSet::It _shape;
  public:
    Site(const RatPoint_2& pos, const StarSet::It &shape)
      : _pos(pos)
      , _shape(shape)
    {};

    static Site from_pos_and_shapename(const RatPoint_2& pos, const std::string& shape, const StarSet& stars);
    static Site from_ipe_element(const pugi::xml_node& node, const StarSet& stars);
    const RatPoint_2& pos() const { return _pos; };
    const Star& shape() const { return _shape->second; };

    void add_to_input(SurfInput& si) const { shape().add_to_input(si, pos()); };

    void make_vertices(std::back_insert_iterator<RatRay3List> verticesIt) const { shape().make_vertices(verticesIt, pos()); };
    void make_triangles(std::back_insert_iterator<RealTriangleList> trianglesIt, const RatNT& max_time) const { shape().make_triangles(trianglesIt, pos(), max_time); };
    void add_to_input(std::back_insert_iterator<TriangleList> trianglesIt, const int site_idx, const RatNT& max_time) const { shape().add_to_input(trianglesIt, pos(), site_idx, max_time); };
};

class SiteSet : private std::vector<Site> {
  private:
    using Base = std::vector<Site>;
    Site get_one_from_line(const std::string& line, const StarSet& stars);
  public:
    void load_from_pnt(std::istream &ins, const StarSet& stars);
    void load_from_line(std::istream &ins, const StarSet& stars);
    void load_from_ipe(std::istream &ins, const StarSet& stars);

    RatNT get_closest_distance_squared() const;

    SurfInput make_surf_input() const;

    RatRay3List make_vertices() const; /* of unit height */
    RealTriangleList make_triangles() const; /* of unit height */
    TriangleList make_vd_input(const RatNT& max_time) const;

    using Base::size;
    using Base::begin;
    using Base::end;

    unsigned total_size() const {
      return accumulate(begin(), end(), 0,
        [] (unsigned cnt, const Site& s) { return cnt + s.shape().size(); });
    }
};

class Input {
  private:
    StarSet _stars;
    SiteSet _sites;
    StagesPtr _stages;
    StatsPtr _stats;

    void preprocess();
  public:
    Input(const std::string &stars_fn, const std::string &sites_fn, double random_scale_sigma, StarFormat star_fmt, SiteFormat site_fmt, StagesPtr stages, StatsPtr stats);
    bool do_sk(std::ostream &os, std::string skoffset) const;
    bool do_vd(std::ostream &os, const RatNT& max_time, std::string skoffset) const;
    const SiteSet& sites() const { return _sites; };
};
