#pragma once

#include "stardist.h"

#include <unordered_map>
#include <pugixml.hpp>

RatNT string2RatNT(const std::string& s);

class SurfInput;

class Star {
  private:
    std::vector<RatPoint_2> _pts;
  public:
    Star(const std::vector<RatPoint_2> pts, const RatPoint_2& center, const std::string& stroke /* for logging/warning purposes only */);
    RatNT get_max_distance_squared() const;
    void add_to_distance_set(std::set<CoreNT>& distances) const;
    const std::vector<RatPoint_2>& pts() const { return _pts; };
    void shrink(const RatNT& scale);
    void add_to_input(SurfInput& si, const RatPoint_2& p) const;
    void add_to_input(TriangleList& triangles, const RatPoint_2& p, const int site_idx, const RatNT& max_time) const;
};

class StarSet : private std::unordered_map<std::string, Star> {
  private:
    using Base = std::unordered_map<std::string, Star>;
  public:
    using It = Base::const_iterator;

    void load_from_ipe(std::istream &ins);

    using Base::find;
    using Base::end;

    RatNT get_max_distance_squared() const;
    CoreNT get_closest_distance() const;
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
    void add_to_input(SurfInput& si) const;
    void add_to_input(TriangleList& triangles, const int site_idx, const RatNT& max_time) const;
};

class SiteSet {
  private:
    std::vector<Site> sites;
  public:
    void load_from_ipe(std::istream &ins, const StarSet& stars);

    RatNT get_closest_distance_squared() const;
    SurfInput make_surf_input() const;
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
