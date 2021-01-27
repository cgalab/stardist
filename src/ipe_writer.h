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

class SkeletonDCEL;
class StarVD;

class IpeWriter {
  static const std::string STROKE_INPUT;
  static const std::string STROKE_ARC;
  static const std::string STROKE_ARC_INTERNAL;
  static const std::string STROKE_OFFSET;
  static const std::string STROKE_UNKNOWN;

  private:
    template<typename Segment> static void write_segment(std::ostream& os, const Segment& s, const std::string &stroke);
    static void write_polygon(std::ostream& os, const std::vector<RatPoint_2>& pts, const std::string &stroke);
    static void write_polygons(std::ostream& os, const std::vector<std::vector<RatPoint_2>>& polys, const std::string &stroke);

    static void write_header(std::ostream& os);
    static void write_footer(std::ostream& os);
    static void write_sites(std::ostream& os, const SiteSet& sites);

    static void write_vd_arrangement_faces(std::ostream& os, const StarVD& vd);

  public:
    static void write_skeleton(std::ostream& os, const SkeletonDCEL& sk, const SiteSet& sites, const std::string& offset_spec);
    static void write_vd(std::ostream& os, const StarVD& vd, const SiteSet& sites, const std::string& offset_spec);
};
