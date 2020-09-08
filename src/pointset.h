#pragma once

#include "stardist.h"

#include <vector>
#include <unordered_map>
#include <pugixml.hpp>

class SurfInput;

class Star {
  private:
    std::vector<Point_2> pts;
  public:
    Star(const std::vector<Point_2> pts, const Point_2& center);
    NT get_max_distance_squared() const;
    void shrink(const NT& scale);
    void add_to_input(SurfInput& si, const Point_2& p) const;
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
    const Point_2 pos_;
    const StarSet::It shape_;
  public:
    Site(const Point_2& pos, const StarSet::It &shape)
      : pos_(pos)
      , shape_(shape)
    {};

    static Site from_ipe_element(const pugi::xml_node& node, const StarSet& stars);
    const Point_2& pos() const { return pos_; };
    const Star& shape() const { return shape_->second; };
    void add_to_input(SurfInput& si) const;
};

class SiteSet {
  private:
    std::vector<Site> sites;
  public:
    void load_from_ipe(std::istream &ins, const StarSet& stars);

    NT get_closest_distance_squared() const;
    SurfInput make_surf_input() const;
};

class Input {
  private:
    StarSet stars;
    SiteSet sites;
  public:
    Input(std::istream &stars_ipe, std::istream &sites_ipe);
    void do_sk(std::ostream &os, bool write_ipe, std::string skoffset);
};
