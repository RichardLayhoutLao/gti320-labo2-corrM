cmake_minimum_required(VERSION 3.15)

project(labo01)

# Add googletest
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC) 
  # Configure for pre-compiled installation of Google Test library
  set(GTEST_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../extern/googletest)
  set(GTEST_LIBRARY ${GTEST_SOURCE_DIR}/lib/gtest.lib)
  set(GTEST_MAIN_LIBRARY ${GTEST_SOURCE_DIR}/lib/gtest_main.lib)
  set(GTEST_LIBRARY_DEBUG ${GTEST_SOURCE_DIR}/lib/gtestd.lib)
  set(GTEST_MAIN_LIBRARY_DEBUG ${GTEST_SOURCE_DIR}/lib/gtest_maind.lib)
  set(GTEST_INCLUDE_DIR ${GTEST_SOURCE_DIR}/include)
  INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIR} ${COMMON_INCLUDES})
  add_definitions(-DGTEST_HAS_PTHREAD=0)
  add_library(gtest STATIC IMPORTED)
  set_target_properties(gtest PROPERTIES IMPORTED_LOCATION_DEBUG ${GTEST_LIBRARY_DEBUG})
  set_target_properties(gtest PROPERTIES IMPORTED_LOCATION_RELEASE ${GTEST_LIBRARY})
  set(GTEST_FOUND)
endif() 

find_package(GTest REQUIRED)

# Add .cpp and .h files
set(HEADERS DenseStorage.h MatrixBase.h Matrix.h Math3D.h Vector.h Operators.h)
set(SOURCE main.cpp)
add_executable(labo01 ${SOURCE} ${HEADERS})

# Add linking information for Google Test
target_link_libraries(labo01 gtest)

if(MSVC)
# Pre-compiled version of the library uses MultiThreaded static runtime library
set_property(TARGET labo01 PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()


