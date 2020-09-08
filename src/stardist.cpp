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

#include <surf.h>
#include "stardist.h"
#include "pointset.h"

static const char* short_options = "hIv:";
static struct option long_options[] = {
  { "help"               , no_argument        , 0, 'h'},
  { "ipe"                , no_argument        , 0, 'I'},
  { "verbose"            , optional_argument  , 0, 'v'},
  { 0, 0, 0, 0}
};

[[noreturn]]
static void
usage(const char *progname, int err) {
  FILE *f = err ? stderr : stdout;

  fprintf(f, "Usage: %s <STARSET> [<POINTSET> [<OUTPUT>]]\n", progname);
  fprintf(f,"        --verbose      Be slightly verbose\n");
  fprintf(f,"        --ipe          Produce an IPE file as output.\n");
  fprintf(f,"\n");
  fprintf(f,"        STARSET  .ipe file -- stars are polygons with their center as an IPE marker\n");
  fprintf(f,"        POINTSET .ipe file -- sites are IPE markers\n");
  exit(err);
}

int
main(int argc, char *argv[]) {
  NT dummy1 = NT::getOne();
  NT dummy2 = NT::getZero();

  bool ipe = false;
  unsigned verbose = 0;

  while (1) {
    int option_index = 0;
    int r = getopt_long(argc, argv, short_options, long_options, &option_index);

    if (r == -1) break;
    switch (r) {
      case 'h':
        usage(argv[0], 0);
        break;

      case 'I':
        ipe = true;
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

      default:
        std::cerr << "Invalid option " << (char)r << std::endl;
        exit(1);
    }
  }

  setup_logging(argc, argv);
  el::Loggers::setVerboseLevel(verbose);

  int numargs = argc - optind;
  if (numargs < 1 || numargs > 3) {
    usage(argv[0], 1);
  }

  std::istream *starin = &std::cin;
  std::ifstream starstreamin;
  {
    std::string fn(argv[optind + 0]);
    if (fn != "-") {
      starstreamin.open(fn);
      starin = &starstreamin;
    }
  }

  std::istream *in = &std::cin;
  std::ifstream filestreamin;
  if (argc - optind >= 2) {
    std::string fn(argv[optind + 1]);
    if (fn != "-") {
      filestreamin.open(fn);
      in = &filestreamin;
    }
  }

  std::ostream *out = &std::cout;
  std::ofstream filestreamout;
  if (numargs >= 3) {
    std::string fn(argv[optind + 2]);
    if (fn != "-") {
      filestreamout.open(fn);
      out = &filestreamout;
    }
  }

  StarSet stars;
  stars.load_from_ipe(*starin);

  SiteSet sites(stars);
  sites.load_from_ipe(*in);

  exit(0);
}
