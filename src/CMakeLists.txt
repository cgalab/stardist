project (stardist)


set_source_files_properties(easyloggingpp/src/easylogging++.cc PROPERTIES COMPILE_FLAGS "-Wno-float-equal -Wno-missing-noreturn")

find_package( Boost REQUIRED iostreams )
if ( NOT Boost_FOUND )
  message(FATAL_ERROR "Boost not found.")
endif()
include_directories(${Boost_INCLUDE_DIRS})

find_package( CGAL COMPONENTS Core )
if ( NOT CGAL_FOUND )
  message(FATAL_ERROR "CGAL not found.")
endif()
set (CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE 1)
set ( CGAL_BUILD_TYPE_INIT ${CMAKE_BUILD_TYPE} )
include( ${CGAL_USE_FILE} )

find_package( pugixml )

find_library(SURF_LIBRARY
    NAMES libsurflib.so libsurflib.a
    HINTS "${SURF_LIBRARY_PATH}" "${SURF_LIBRARY_PATH}/surf"
)
find_path(SURF_INCLUDE
  NAMES surf.h
  PATHS "${SURF_INCLUDE_PATH}/include" "${SURF_INCLUDE_PATH}/surf/include"
)
foreach(_var IN ITEMS SURF_LIBRARY SURF_INCLUDE_DIR)
    get_property(_var_cached CACHE ${_var} PROPERTY VALUE)
    message(STATUS "CACHED path: ${_var_cached}")
    message(STATUS "path: ${${_var}}")
endforeach()
if ( NOT SURF_LIBRARY )
  message(FATAL_ERROR "surfer library not found.")
endif()
if ( NOT SURF_INCLUDE )
  message(FATAL_ERROR "surfer header files not found.")
endif()
include_directories("${SURF_INCLUDE}" "${SURF_INCLUDE}/../src" "${SURF_LIBRARY_PATH}" "${SURF_LIBRARY_PATH}/..")

add_executable(stardist
                  stardist.cpp

                  pointset.cpp
                  vd.cpp
                  ipe_writer.cpp

                  easyloggingpp/src/easylogging++.cc

                  "${PROJECT_BINARY_DIR}/gitversion.h"
               )

target_link_libraries(stardist ${SURF_LIBRARY})
target_link_libraries(stardist ${Boost_LIBRARIES} )
target_link_libraries(stardist ${CGAL_LIBRARIES})
target_link_libraries(stardist pugixml)


message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}. CGAL_USE_FILE is ${CGAL_USE_FILE}" )
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}." )

set(GITVERSION_DEPENDS "${CMAKE_SOURCE_DIR}/.git/logs/HEAD")
if (NOT EXISTS "${GITVERSION_DEPENDS}")
  message(STATUS "Not a git checkout")
  set(GITVERSION_DEPENDS "")
endif()

add_custom_command(
  OUTPUT "${PROJECT_BINARY_DIR}/gitversion.h"
  DEPENDS "${GITVERSION_DEPENDS}"
  COMMAND ${CMAKE_COMMAND}
    -Dlocal_dir="${CMAKE_SOURCE_DIR}"
    -Doutput_dir="${PROJECT_BINARY_DIR}"
    -P "${CMAKE_SOURCE_DIR}/cmake/gitversion.cmake"
)
