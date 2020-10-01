#pragma once

#include "stardist.h"

class StarVD {
  private:
    TriangleList _triangles;
    Envelope_diagram_2 _arr;

    bool check_sufficiently_far();

    bool _is_valid = false;

    RatNT _max_time;
  public:
    StarVD(const SiteSet& sites, const StarSet& stars, const RatNT& max_time);

    bool is_valid() const { return _is_valid; };

    const TriangleList& triangles() const { return _triangles; };
    const Envelope_diagram_2& arr() const { return _arr; };
};
