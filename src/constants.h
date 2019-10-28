#pragma once

#include <string>
#include <chrono>

#define constexpr_string inline const std::string

constexpr_string  PATH_ROOT = "";
constexpr_string  PATH_FUZZY_OBS_AVOID = PATH_ROOT + "assets/data/fuzzy-obs-avoid.fll";
constexpr_string  PATH_FUZZY_SIMPLE_NAVIGATOR = PATH_ROOT + "assets/data/simpleNavigator.fll";
constexpr_string  PATH_FONT_CONSOLAS = PATH_ROOT + "assets/data/consolas.ttf";

constexpr auto    RUN_FREQ_MS = std::chrono::milliseconds(10); // ms
constexpr auto    MAX_DIST_TO_OBSTACLE = 0.7f; // meters
constexpr auto    LIDAR_RANGE_LIMIT = 10; // number of rays
// constexpr auto    LIDAR_RANGE_F = { 0.19, -0.19 }; // rad
// constexpr auto    LIDAR_RANGE_L = { 1.76,  1.37 }; // rad
// constexpr auto    LIDAR_RANGE_R = {-1.37, -1.76 }; // rad
constexpr auto    FUZZY_SCALING_FACTOR = 1.0f;

constexpr auto    WNDW_CAMERA   = "camera";
constexpr auto    WNDW_LIDAR    = "lidar";
constexpr auto    WNDW_DEBUG    = "debug";
constexpr auto	  WNDW_HANDLES  = { WNDW_DEBUG, WNDW_LIDAR, WNDW_CAMERA };
constexpr size_t  WNDW_WIDTHS[] = { 700, 400, 200 };
constexpr size_t  WNDW_ORIGIN[] = { 100, 50 };
constexpr auto    WNDW_MARGIN   = 20;