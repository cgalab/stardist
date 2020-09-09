#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include "../easyloggingpp/src/easylogging++.h"

using Kernel  = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
using NT = Kernel::FT;

using Line_2    = typename Kernel::Line_2;
using Point_2   = typename Kernel::Point_2;
using Segment_2 = typename Kernel::Segment_2;
using Vector_2  = typename Kernel::Vector_2;

class SkeletonDCEL;
void skeleton_write_ipe(std::ostream& os, const SkeletonDCEL& sk, const std::string& offset_spec);
