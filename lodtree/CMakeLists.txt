# bump version here
set(lodtree_VERSION 1.0)

define_module(LIBRARY lodtree=${lodtree_VERSION}
  DEPENDS roarchive>=1.3 geo>=1.26 imgproc>=1.20
  math>=1.2 utility>=1.19 dbglog>=1.4
  Boost_FILESYSTEM
  )

set(lodtree_SOURCES
  lodtreefile.hpp lodtreefile.cpp
)

add_library(lodtree STATIC ${lodtree_SOURCES})
buildsys_library(lodtree)

target_link_libraries(lodtree ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(lodtree ${MODULE_DEFINITIONS})
