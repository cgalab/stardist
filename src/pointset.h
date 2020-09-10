#pragma once

#include "stardist.h"

#include <unordered_map>
#include <pugixml.hpp>

class SurfInput;

class Star {
  private:
    std::vector<Point_2> _pts;
  public:
    Star(const std::vector<Point_2> pts, const Point_2& center);
    NT get_max_distance_squared() const;
    const std::vector<Point_2>& pts() const { return _pts; };
    void shrink(const NT& scale);
    void add_to_input(SurfInput& si, const Point_2& p) const;
    void add_to_input(TriangleList& triangles, const Point_2& p, const int site_idx, const NT& max_time) const;
};

class StarSet : private std::unordered_map<std::string, Star> {
  private:
    using Base = std::unordered_map<std::string, Star>;
  public:
    using It = Base::const_iterator;

    void load_from_ipe(std::istream &ins);

    using Base::find;
    using Base::end;

    NT get_max_distance_squared() const;
    void shrink(const NT& scale);
};

class Site {
  private:
    const Point_2 _pos;
    const StarSet::It _shape;
  public:
    Site(const Point_2& pos, const StarSet::It &shape)
      : _pos(pos)
      , _shape(shape)
    {};

    static Site from_ipe_element(const pugi::xml_node& node, const StarSet& stars);
    const Point_2& pos() const { return _pos; };
    const Star& shape() const { return _shape->second; };
    void add_to_input(SurfInput& si) const;
    void add_to_input(TriangleList& triangles, const int site_idx, const NT& max_time) const;
};

class SiteSet {
  private:
    std::vector<Site> sites;
  public:
    void load_from_ipe(std::istream &ins, const StarSet& stars);

    NT get_closest_distance_squared() const;
    SurfInput make_surf_input() const;
    TriangleList make_vd_input(const NT& max_time) const;
    const std::vector<Site>& get_sites() const { return sites; };
};

class Input {
  private:
    StarSet stars;
    SiteSet sites;
  public:
    Input(std::istream &stars_ipe, std::istream &sites_ipe);
    bool do_sk(std::ostream &os, std::string skoffset) const;
    bool do_vd(std::ostream &os, const NT& max_time, bool auto_height, std::string skoffset) const;
};
