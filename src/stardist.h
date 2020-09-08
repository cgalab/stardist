#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include "../easyloggingpp/src/easylogging++.h"

using Kernel  = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
using NT = Kernel::FT;

using Point_2 = typename Kernel::Point_2;
using Vector_2 = typename Kernel::Vector_2;
