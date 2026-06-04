cmake_minimum_required(VERSION 3.14)
project(mcap_converter CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── vehicle_can_decoder package root (two directories up from here) ───────────
get_filename_component(VCD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

include(FetchContent)

# Bridge env vars to FetchContent cache variables (see main CMakeLists.txt).
foreach(_dep dbcppp exprtk)
  string(TOUPPER ${_dep} _dep_uc)
  if(DEFINED ENV{FETCHCONTENT_SOURCE_DIR_${_dep_uc}})
    set(FETCHCONTENT_SOURCE_DIR_${_dep_uc} "$ENV{FETCHCONTENT_SOURCE_DIR_${_dep_uc}}")
  endif()
endforeach()
if(DEFINED ENV{FETCHCONTENT_FULLY_DISCONNECTED})
  set(FETCHCONTENT_FULLY_DISCONNECTED "$ENV{FETCHCONTENT_FULLY_DISCONNECTED}")
endif()

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
# Requires: source /opt/ros/humble/setup.bash
#           colcon build --packages-select vehicle_can_decoder
#           source install/setup.bash
if(NOT DEFINED ENV{AMENT_PREFIX_PATH})
  message(WARNING
    "ROS 2 environment not sourced. Run: source /opt/ros/humble/setup.bash")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(vehicle_can_decoder REQUIRED)

if(NOT vehicle_can_decoder_FOUND)
  message(FATAL_ERROR
    "vehicle_can_decoder not found. Build order:\n"
    "  colcon build --packages-select vehicle_can_decoder\n"
    "  source install/setup.bash")
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
# DbcDecoder / SignalRouter / SignalTransformer have no ROS 2 dependency and are
# compiled directly from source. VehicleCanNode (rclcpp) is intentionally excluded.
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
  yaml-cpp::yaml-cpp
)
ament_target_dependencies(mcap_converter
  rosbag2_cpp
  rosbag2_storage
  rclcpp
  can_msgs
  vehicle_can_decoder
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS mcap_converter DESTINATION bin)
