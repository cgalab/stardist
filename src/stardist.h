#pragma once

#include <vector>

#include "easyloggingpp/src/easylogging++.h"

#include "../starconfig.h"

#ifdef VD_ONLY
  #define RATIONAL_NT
#endif

#ifdef RATIONAL_NT
  #include <CGAL/Exact_rational.h>
  #include <CGAL/Cartesian.h>
  using NT = CGAL::Exact_rational;
  using Kernel  =  CGAL::Cartesian<NT>;
  // #define NT_CAN_PARSE_DECIMALS

  static const NT CORE_ONE =  1;
  static const NT CORE_ZERO = 0;
#else
  #include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
  using Kernel  = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
  using NT = Kernel::FT;
 #define NT_CAN_PARSE_DECIMALS

  static const NT& CORE_ONE =  NT::getOne();
  static const NT& CORE_ZERO = NT::getZero();
#endif

#include <CGAL/Env_triangle_traits_3.h>
#include <CGAL/Env_surface_data_traits_3.h>
#include <CGAL/envelope_3.h>

using Line_2             = typename Kernel::Line_2;
using Point_2            = typename Kernel::Point_2;
using Segment_2          = typename Kernel::Segment_2;
using Vector_2           = typename Kernel::Vector_2;

using Line_3             = typename Kernel::Line_3;
using Plane_3            = typename Kernel::Plane_3;
using Point_3            = typename Kernel::Point_3;
using Vector_3           = typename Kernel::Vector_3;

using Traits_3           = typename CGAL::Env_triangle_traits_3<Kernel>;
using Triangle_3         = typename Traits_3::Surface_3;
using Data_traits_3      = typename CGAL::Env_surface_data_traits_3<Traits_3, std::pair<int,int> >;
using Data_triangle_3    = typename Data_traits_3::Surface_3;
using Envelope_diagram_2 = typename CGAL::Envelope_diagram_2<Data_traits_3>;

using TriangleList       = typename std::vector<Data_triangle_3>;

class SiteSet;
