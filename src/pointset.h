#pragma once

#include "stardist.h"

#include <vector>
#include <unordered_map>
#include <pugixml.hpp>

class Star {
  private:
    std::vector<Point_2> pts;
  public:
    Star(const std::vector<Point_2> pts, const Point_2& center);
};

class StarSet : private std::unordered_map<std::string, Star> {
  private:
    using Base = std::unordered_map<std::string, Star>;
  public:
    using It = Base::const_iterator;

    void load_from_ipe(std::istream &ins);

    using Base::find;
    using Base::end;
};

class Site {
  private:
    const Point_2 pos;
    const StarSet::It shape;
  public:
    Site(const Point_2& p_pos, const StarSet::It &p_shape)
      : pos(p_pos)
      , shape(p_shape)
    {};

    static Site from_ipe_element(const pugi::xml_node& node, const StarSet& stars);
};

class SiteSet {
  private:
    const StarSet& stars;
    std::vector<Site> sites;
  public:
    SiteSet(const StarSet& p_stars)
      : stars(p_stars)
      {};
    void load_from_ipe(std::istream &ins);
};
