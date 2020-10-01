#pragma once

#include "../starconfig.h"

//#include <CGAL/Exact_rational.h>
//using RatNT = CGAL::Exact_rational;

#include <CGAL/CORE_BigRat.h>
using RatNT = CORE::BigRat;

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Env_triangle_traits_3.h>
#include <CGAL/Env_surface_data_traits_3.h>
#include <CGAL/envelope_3.h>

#include <vector>

#include "easyloggingpp/src/easylogging++.h"

using RatKernel  =  CGAL::Simple_cartesian<RatNT>;

using CoreKernel  = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
using CoreNT = CoreKernel::FT;

static const CoreNT& Core_ONE =  CoreNT::getOne();
static const CoreNT& Core_ZERO = CoreNT::getZero();


using CorePoint_2        = typename CoreKernel::Point_2;
using CoreSegment_2      = typename CoreKernel::Segment_2;
using CoreRay_2          = typename CoreKernel::Ray_2;

using CoreSegment_3      = typename CoreKernel::Segment_3;
using CoreRay_3          = typename CoreKernel::Ray_3;



using RatLine_2          = typename RatKernel::Line_2;
using RatPoint_2         = typename RatKernel::Point_2;
using RatSegment_2       = typename RatKernel::Segment_2;
using RatVector_2        = typename RatKernel::Vector_2;

using RatLine_3          = typename RatKernel::Line_3;
using RatPlane_3         = typename RatKernel::Plane_3;
using RatPoint_3         = typename RatKernel::Point_3;
using RatVector_3        = typename RatKernel::Vector_3;

using Traits_3           = typename CGAL::Env_triangle_traits_3<RatKernel>;
using Triangle_3         = typename Traits_3::Surface_3;
using Data_traits_3      = typename CGAL::Env_surface_data_traits_3<Traits_3, std::pair<int,int> >;
using Data_triangle_3    = typename Data_traits_3::Surface_3;
using Envelope_diagram_2 = typename CGAL::Envelope_diagram_2<Data_traits_3>;

using TriangleList       = typename std::vector<Data_triangle_3>;

class SiteSet;
class StarSet;

inline CorePoint_2
Rat2Core(const RatPoint_2& p) {
  return CorePoint_2(p.x(), p.y());
}
