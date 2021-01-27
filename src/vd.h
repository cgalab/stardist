/**  stardist -- compute skeletal structures based on a star-based distanced distance function
 *
 *  Copyright 2020, 2021 Peter Palfraader
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
