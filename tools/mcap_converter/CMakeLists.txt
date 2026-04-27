cmake_minimum_required(VERSION 3.14)
project(mcap_converter CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── vehicle_can_decoder package root (two directories up from here) ───────────
get_filename_component(VCD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

include(FetchContent)

# ── Foxglove MCAP C++ library (header-only, no ROS2 dependency) ───────────────
# Populated without processing the top-level CMakeLists.txt so we avoid
# pulling in unrelated targets from the monorepo.
FetchContent_Declare(
  mcap
  GIT_REPOSITORY https://github.com/foxglove/mcap.git
  GIT_TAG        releases/cpp/v1.4.1
  GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(mcap)
if(NOT mcap_POPULATED)
  FetchContent_Populate(mcap)
endif()

add_library(mcap INTERFACE)
target_include_directories(mcap INTERFACE "${mcap_SOURCE_DIR}/cpp/mcap/include")

# Optional compression support for reading lz4 / zstd compressed MCAP files.
# Install system packages to enable: sudo apt install liblz4-dev libzstd-dev
find_library(LZ4_LIB  NAMES lz4)
find_path(LZ4_INC     NAMES lz4.h)
if(LZ4_LIB AND LZ4_INC)
  target_compile_definitions(mcap INTERFACE MCAP_COMPRESSION_LZ4)
  target_link_libraries(mcap INTERFACE ${LZ4_LIB})
  target_include_directories(mcap INTERFACE ${LZ4_INC})
  message(STATUS "mcap_converter: LZ4 compression enabled")
else()
  message(STATUS "mcap_converter: LZ4 not found -- lz4-compressed bags unsupported")
endif()

find_library(ZSTD_LIB NAMES zstd)
find_path(ZSTD_INC    NAMES zstd.h)
if(ZSTD_LIB AND ZSTD_INC)
  target_compile_definitions(mcap INTERFACE MCAP_COMPRESSION_ZSTD)
  target_link_libraries(mcap INTERFACE ${ZSTD_LIB})
  target_include_directories(mcap INTERFACE ${ZSTD_INC})
  message(STATUS "mcap_converter: ZSTD compression enabled")
else()
  message(STATUS "mcap_converter: ZSTD not found -- zstd-compressed bags unsupported")
endif()

# ── dbcppp (same pin as the main package) ────────────────────────────────────
FetchContent_Declare(
  dbcppp
  GIT_REPOSITORY https://github.com/xR3b0rn/dbcppp.git
  GIT_TAG        v3.2.6
  GIT_SHALLOW    TRUE
)
set(dbcppp_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(dbcppp_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(dbcppp)
target_include_directories(libdbcppp PUBLIC
  $<BUILD_INTERFACE:${dbcppp_SOURCE_DIR}/include>
)

# ── exprtk (same pin as the main package, header-only) ───────────────────────
FetchContent_Declare(
  exprtk
  GIT_REPOSITORY https://github.com/ArashPartow/exprtk.git
  GIT_TAG        0.0.3
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(exprtk)
add_library(exprtk INTERFACE)
target_include_directories(exprtk INTERFACE ${exprtk_SOURCE_DIR})

# ── yaml-cpp (system package) ─────────────────────────────────────────────────
find_package(yaml-cpp REQUIRED)
# yaml-cpp 0.7 exports "yaml-cpp"; 0.8+ exports "yaml-cpp::yaml-cpp".
# Create an alias so the executable target below works with either version.
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

# ── vehicle_can_decoder non-ROS core (compiled inline) ────────────────────────
# Only the three ROS-independent components are compiled here.
# VehicleCanNode (rclcpp) is intentionally excluded.
add_library(vcd_core STATIC
  ${VCD_ROOT}/src/dbc_decoder.cpp
  ${VCD_ROOT}/src/signal_transformer.cpp
  ${VCD_ROOT}/src/signal_router.cpp
)
target_include_directories(vcd_core PUBLIC ${VCD_ROOT}/include)
target_link_libraries(vcd_core PRIVATE libdbcppp exprtk)
target_compile_options(vcd_core PRIVATE $<$<CXX_COMPILER_ID:GNU>:-O2>)

# ── mcap_converter executable ─────────────────────────────────────────────────
add_executable(mcap_converter src/mcap_converter.cpp)
target_link_libraries(mcap_converter
  vcd_core
  mcap
  yaml-cpp::yaml-cpp
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS mcap_converter DESTINATION bin)
