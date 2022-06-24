# bump version here
set(lodtree_VERSION 1.1)

define_module(LIBRARY lodtree=${lodtree_VERSION}
  DEPENDS
  BuildSystem>=1.13
  roarchive>=1.3 geo>=1.26
  math>=1.2 utility>=1.19 dbglog>=1.4
  Boost_FILESYSTEM
  )

set(lodtree_SOURCES
  lodtreefile.hpp lodtreefile.cpp
)

add_library(lodtree STATIC ${lodtree_SOURCES})
buildsys_library(lodtree)

target_link_libraries(lodtree ${MODULE_LIBRARIES})
target_compile_definitions(lodtree PRIVATE ${MODULE_DEFINITIONS})
