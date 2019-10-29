#pragma once

#include <string>
#include <chrono>
#include <initializer_list>

#define constexpr_string inline const std::string

constexpr_string  PATH_ROOT = "";
constexpr_string  PATH_FUZZY_OBS_AVOID = PATH_ROOT + "assets/data/fuzzy-obs-avoid.fll";
constexpr_string  PATH_FUZZY_SIMPLE_NAVIGATOR = PATH_ROOT + "assets/data/simpleNavigator.fll";
constexpr_string  PATH_FONT_CONSOLAS = PATH_ROOT + "assets/data/consolas.ttf";

constexpr auto    RUN_FREQ_MS = std::chrono::milliseconds(10); // ms
constexpr auto    MAX_DIST_TO_OBSTACLE = 0.7f; // meters
constexpr auto    GOAL_POS = { 4.f, -3.f, 0.f }; // meters
constexpr auto    LIDAR_RANGE_LIMIT = 10; // number of rays
constexpr auto    FUZZY_SCALING_FACTOR = 1.0f;

constexpr auto    WNDW_CAMERA   = "camera";
constexpr auto    WNDW_LIDAR    = "lidar";
constexpr auto    WNDW_DEBUG    = "debug";
constexpr auto	  WNDW_HANDLES  = { WNDW_DEBUG, WNDW_LIDAR, WNDW_CAMERA };
constexpr size_t  WNDW_WIDTHS[] = { 700, 400, 200 };
constexpr size_t  WNDW_ORIGIN[] = { 100, 50 };
constexpr auto    WNDW_MARGIN   = 20;

constexpr auto    ERR_NOT_INIT  = "System is not initialized.";
constexpr auto    ERR_RE_INIT   = "System is already initialized.";