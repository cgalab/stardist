project (stardist)

find_package( Boost REQUIRED iostreams )
if ( NOT Boost_FOUND )
  message(STATUS "Boost not found.")
  return()
endif()
include_directories(${Boost_INCLUDE_DIRS})

add_executable(stardist
                  stardist.cpp
               )
