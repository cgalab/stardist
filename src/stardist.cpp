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

static const char* short_options = "hs:c:Ivo";
static struct option long_options[] = {
  { "help"               , no_argument        , 0, 'h'},
  { "ipe"                , no_argument        , 0, 'I'},
  { 0, 0, 0, 0}
};

[[noreturn]]
static void
usage(const char *progname, int err) {
  FILE *f = err ? stderr : stdout;

  fprintf(f, "Usage: %s <DB> [<OUTPUT>]\n", progname);
  fprintf(f,"        --verbose      Be slightly verbose\n");
  fprintf(f,"        --ipe          Produce an IPE file as output.\n");
  exit(err);
}

int
main(int argc, char *argv[]) {
  bool ipe = false;
  bool verbose = false;

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
        verbose = true;
        break;

      default:
        std::cerr << "Invalid option " << (char)r << std::endl;
        exit(1);
    }
  }

  int numargs = argc - optind;
  if (numargs < 1 || numargs > 2) {
    usage(argv[0], 1);
  }


  std::ostream *out = &std::cout;
  std::ofstream filestreamout;

  std::string databasefn(argv[optind]);
  if (numargs >= 2) {
    std::string fn(argv[optind + 1]);
    if (fn != "-") {
      filestreamout.open(fn);
      out = &filestreamout;
    }
  }

}
