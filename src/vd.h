#pragma once

#include "stardist.h"

class StarVD {
  private:
    TriangleList _triangles;
    Envelope_diagram_2 _arr;

    bool check_sufficiently_far_approximately();

    bool _is_valid = false;

    RatNT _max_time;
    StagesPtr stages;
  public:
    StarVD(const SiteSet& sites, const StarSet& stars, const RatNT& max_time, StagesPtr stages);

    bool is_valid() const { return _is_valid; };
    RatNT guess_upper_bound(const SiteSet& sites, const StarSet& stars) const;
    RatNT find_last_pierce_event(const SiteSet& sites, const StarSet& stars) const;

    const TriangleList& triangles() const { return _triangles; };
    const Envelope_diagram_2& arr() const { return _arr; };
};
