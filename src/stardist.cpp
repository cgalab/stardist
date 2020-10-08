/** Copyright 2020 Peter Palfrader
 */

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <assert.h>

#include <fstream>
#include <vector>
#include <iostream>
#include <string.h>

#include "stardist.h"
#include <surf.h>
// INITIALIZE_EASYLOGGINGPP  /* done by the surfer library */

#include "pointset.h"
#include "gitversion.h"

#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>

#include <sys/resource.h>

static const char* short_options = "hv";
static struct option long_options[] = {
  { "help"               , no_argument        , 0, 'h'},
  { "vd"                 , no_argument        , 0, 'V'},
  { "verbose"            , optional_argument  , 0, 'v'},
  { "sk-offset"          , required_argument  , 0, 'O'},
  { "vd-height"          , required_argument  , 0, 'H'},
  { "stats-fd"           , required_argument  , 0, 'S'},
  { "site-format"        , required_argument  , 0, 'F'},
  { "random-scale"       , required_argument  , 0, 'R'},
  { "random-seed"        , required_argument  , 0, 'X'},
  { 0, 0, 0, 0}
};

std::unique_ptr<std::mt19937> RandomGenerator::g = NULL;

static void
star_setup_logging(int argc, char* argv[]) {
  START_EASYLOGGINGPP(argc, argv);

  el::Configurations defaultConf;
  defaultConf.setGlobally(el::ConfigurationType::Format, "%datetime{%H%m%s.%g} %levshort %msg");
  el::Loggers::reconfigureAllLoggers(defaultConf);
  el::Loggers::addFlag( el::LoggingFlag::ColoredTerminalOutput );
}

[[noreturn]]
static void
usage(const char *progname, int err) {
  FILE *f = err ? stderr : stdout;

  fprintf(f, "Usage: %s <STARSET> [<POINTSET> [<OUTPUT>]]\n", progname);
  fprintf(f,"  Options: --verbose      Be slightly verbose\n");
  fprintf(f,"           --vd           Make a VD instead of a straight skeleton/SEVD.\n");
  fprintf(f,"           --sk-offset=<offset-spec>  Draw offsets.\n");
  fprintf(f,"           --vd-height=<HEIGHT>       Extend upwards surfaces up to HEIGHT.\n");
  fprintf(f,"           --stats-fd=<FD>            Enable and print statistics to FD.\n");
  fprintf(f,"           --site-format=(GUESS|IPE|LINE|PNT) site(pointset) format.\n");
  fprintf(f,"           --random-scale=sigma       Scale stars by e^X where X is a random number from N(0,sigma)\n");
  fprintf(f,"\n");
  fprintf(f,"        STARSET  .ipe file -- stars are polygons with their center as an IPE marker\n");
  fprintf(f,"            (or) directory -- stars are files as .line with their center at the origin\n");
  fprintf(f,"        POINTSET .ipe file -- sites are IPE markers\n");
  fprintf(f,"            if .line or .pnt: star is picked randomly if not provided as 3rd attribute\n");
  fprintf(f,"\n");
  fprintf(f,"  offset-spec = <one-block> [ ',' <one-block> [ ',' ... ] ]\n");
  fprintf(f,"  one-block   = <one-offset> [ '+' <one-offset> [ '+' ... ] ]\n");
  fprintf(f,"  one-offset  = [<cnt> '*' ] <time>\n");
  fprintf(f,"  example: '0.01 + 3*0.025, 0.15' or '10 * 0.025'\n");
  exit(err);
}

int
main(int argc, char *argv[]) {
  bool make_vd = false;
  unsigned verbose = 0;
  std::string skoffset;
  RatNT vd_height = 0;
  int stats_fd = -1;
  SiteFormat site_fmt = SiteFormat::guess;
  double random_scale_sigma = 0;

  while (1) {
    int option_index = 0;
    int r = getopt_long(argc, argv, short_options, long_options, &option_index);

    if (r == -1) break;
    switch (r) {
      case 'h':
        usage(argv[0], 0);
        break;

      case 'V':
        make_vd = true;
        break;

      case 'v':
        if (optarg == 0) {
          ++verbose;
        } else {
          char *end_ptr;
          verbose = strtol(optarg, &end_ptr, 10);
          if (*end_ptr != '\0') {
            std::cerr << "Invalid verbosity value" << optarg << std::endl;
            exit(1);
          }
        }
        break;

      case 'H':
        vd_height = string2RatNT(optarg);
        break;

      case 'O':
        skoffset = std::string(optarg);
        break;

      case 'S':
        {
          char *end_ptr;
          stats_fd = strtol(optarg, &end_ptr, 10);
          if (*end_ptr != '\0' || stats_fd < 0) {
            std::cerr << "Invalid stats-fd " << optarg << "." << std::endl;
            exit(1);
          }
        }
        break;

      case 'F':
        {
          std::string fmt(optarg);
          if (fmt == "GUESS") site_fmt = SiteFormat::guess;
          else if (fmt == "IPE") site_fmt = SiteFormat::ipe;
          else if (fmt == "LINE") site_fmt = SiteFormat::line;
          else if (fmt == "PNT") site_fmt = SiteFormat::pnt;
          else {
            std::cerr << "Invalid site format " << fmt << std::endl;
            exit(1);
          }
        }
        break;

      case 'R':
        {
          char *end_ptr;
          random_scale_sigma = strtod(optarg, &end_ptr);
          if (*end_ptr != '\0') {
            std::cerr << "Invalid random scale value" << optarg << std::endl;
            exit(1);
          }
        }
        break;

      case 'X':
        {
          char *end_ptr;
          int random_seed = strtol(optarg, &end_ptr, 10);
          if (*end_ptr != '\0') {
            std::cerr << "Invalid random seed value" << optarg << std::endl;
            exit(1);
          }
          RandomGenerator().init_rnd(random_seed);
        }
        break;

      default:
        std::cerr << "Invalid option " << (char)r << std::endl;
        exit(1);
    }
  }

  star_setup_logging(argc, argv);
  el::Loggers::setVerboseLevel(verbose);

  LOG(INFO) << "stardist"
            << "; git revision: " << GITVERSION
            << "; CMAKE_BUILD_TYPE: " << STAR_CMAKE_BUILD_TYPE
  ;


  int numargs = argc - optind;
  if (numargs < 1 || numargs > 3) {
    usage(argv[0], 1);
  }

  std::string stars_fn(argv[optind + 0]);
  std::string sites_fn(argc - optind >= 2 ? argv[optind + 1] : "-");

  std::ostream *out = &std::cout;
  std::ofstream filestreamout;
  if (numargs >= 3) {
    std::string fn(argv[optind + 2]);
    if (fn != "-") {
      filestreamout.open(fn);
      out = &filestreamout;
    }
  }

  bool success;
  StagesPtr stages = std::make_shared<StagesList>();
  StatsPtr extra_stats = (stats_fd >= 0) ? std::make_shared<std::stringstream>() : NULL;

  stages->push_back( { "START", 0, clock() } );

  Input input(stars_fn, sites_fn, random_scale_sigma, site_fmt, stages, extra_stats);
  if (make_vd) {
    success = input.do_vd(*out, vd_height, skoffset);
  } else {
    success = input.do_sk(*out, skoffset);
  }
  stages->push_back( { "END", 0, clock() } );
  out->flush();

  if (stats_fd >= 0) {
    boost::iostreams::file_descriptor_sink snk{stats_fd, boost::iostreams::never_close_handle};
    boost::iostreams::stream< boost::iostreams::file_descriptor_sink> stats_os{snk};
    stats_os << std::setprecision(10);

    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) < 0) {
      LOG(ERROR) << "getrusage() failed: " << strerror(errno);
      exit(1);
    }
    stats_os << "[STAR] MAXRSS           " << usage.ru_maxrss << std::endl;
    stats_os << "[STAR] NUM_SITES        " << input.sites().size() << std::endl;
    stats_os << "[STAR] SIZE             " << input.sites().total_size() << std::endl;

    std::string line;
    while (std::getline(*extra_stats, line)) {
      stats_os << "[STAR] " << line << std::endl;
    }

    std::vector<clock_t> clocks;
    clocks.push_back(stages->front().clock);

    for (const auto& stage : *stages) {
      while (clocks.size() <= stage.level) {
        clocks.push_back(clocks.back());
      }
      while (clocks.size() > stage.level+1) {
        clocks.pop_back();
      }
      //LOG(INFO) << "l: " << stage.level << "; s: " << clocks.size();
      assert(clocks.size() == stage.level + 1);

      stats_os << "[STAR] CPUTIME_" << std::setw(30) << std::left << stage.label;
      stats_os << std::setw(3) << std::right << stage.level;
      stats_os << " " << std::fixed << std::setw(14) << ((double) (stage.clock - stages->front().clock))/CLOCKS_PER_SEC << "   ";
      for (unsigned i=1; i<clocks.size(); ++i){
        stats_os << " " << std::fixed << std::setw(14) << "";
      }
      stats_os << " " << std::fixed << std::setw(14) << ((double) (stage.clock - clocks.back()))/CLOCKS_PER_SEC;
      stats_os << std::endl;

      clocks.back() = stage.clock;
    }

    stats_os.flush();
  }


  exit(success ? 0 : 1);
}
