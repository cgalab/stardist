#pragma once

#include "stardist.h"

class StarVD {
  private:
    TriangleList _triangles;
    Envelope_diagram_2 _arr;

    bool check_sufficiently_far();

    bool _is_valid = false;

    NT _max_time;
    const bool _auto_height;
    NT _new_max_time;
  public:
    StarVD(const SiteSet& sites, const NT& max_time, const bool auto_height);

    bool is_valid() const { return _is_valid; };

    const TriangleList& triangles() const { return _triangles; };
    const Envelope_diagram_2& arr() const { return _arr; };
};
