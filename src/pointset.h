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
  public:
    Star(const std::vector<RatPoint_2> pts, const RatPoint_2& center, const std::string& stroke /* for logging/warning purposes only */);
    RatNT get_max_vertex_distance_squared() const;
    RatNT get_min_edge_distance_squared() const;
    // void add_to_distance_set(std::set<CoreNT>& distances) const;
    const std::vector<RatPoint_2>& pts() const { return _pts; };
    void shrink(const RatNT& scale);

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

    void load_from_ipe(std::istream &ins);

    using Base::find;
    using Base::end;

    RatNT get_max_vertex_distance_squared() const;
    RatNT get_min_edge_distance_squared() const;
    void shrink(const RatNT& scale);
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

    static Site from_ipe_element(const pugi::xml_node& node, const StarSet& stars);
    const RatPoint_2& pos() const { return _pos; };
    const Star& shape() const { return _shape->second; };

    void add_to_input(SurfInput& si) const { shape().add_to_input(si, pos()); };

    void make_vertices(std::back_insert_iterator<RatRay3List> verticesIt) const { shape().make_vertices(verticesIt, pos()); };
    void make_triangles(std::back_insert_iterator<RealTriangleList> trianglesIt, const RatNT& max_time) const { shape().make_triangles(trianglesIt, pos(), max_time); };
    void add_to_input(std::back_insert_iterator<TriangleList> trianglesIt, const int site_idx, const RatNT& max_time) const { shape().add_to_input(trianglesIt, pos(), site_idx, max_time); };
};

class SiteSet {
  private:
    std::vector<Site> sites;
  public:
    void load_from_ipe(std::istream &ins, const StarSet& stars);

    RatNT get_closest_distance_squared() const;

    SurfInput make_surf_input() const;

    RatRay3List make_vertices() const; /* of unit height */
    RealTriangleList make_triangles() const; /* of unit height */
    TriangleList make_vd_input(const RatNT& max_time) const;

    const std::vector<Site>& get_sites() const { return sites; };
};

class Input {
  private:
    StarSet stars;
    SiteSet sites;
  public:
    Input(std::istream &stars_ipe, std::istream &sites_ipe);
    bool do_sk(std::ostream &os, std::string skoffset) const;
    bool do_vd(std::ostream &os, const RatNT& max_time, std::string skoffset) const;
};
