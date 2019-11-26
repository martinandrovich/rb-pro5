#pragma once

#include <string>
#include <chrono>
#include <initializer_list>

#define constexpr_string inline const std::string


// paths

constexpr_string  PATH_ROOT = "";
constexpr_string  PATH_FUZZY_OBS_AVOID = PATH_ROOT + "assets/data/fuzzy-obs-avoid.fll";
constexpr_string  PATH_FUZZY_SIMPLE_NAVIGATOR = PATH_ROOT + "assets/data/simpleNavigator.fll";
constexpr_string  PATH_FONT_CONSOLAS = PATH_ROOT + "assets/data/consolas.ttf";
constexpr_string  PATH_FLOOR_PLAN = PATH_ROOT + "assets/img/floor_plan.png";

// system config

constexpr auto    RUN_FREQ_MS = std::chrono::milliseconds(20); // ms
constexpr auto    MAX_DIST_TO_OBSTACLE = 0.7f; // meters
constexpr auto    GOAL_POS = { 30.f, 0.f, 0.f }; // meters
constexpr auto    LIDAR_RANGE_LIMIT = 10; // number of rays
constexpr auto    FUZZY_SCALING_FACTOR = 1.0f;

constexpr auto    USE_LOCALIZATION = true;
constexpr auto    USE_OBS_AVOID    = true;

// windows

constexpr auto    WNDW_CAMERA   = "camera";
constexpr auto    WNDW_ESTIMATE = "pos est";
constexpr auto    WNDW_LIDAR    = "lidar";
constexpr auto    WNDW_DEBUG    = "debug";
constexpr auto	  WNDW_HANDLES  = { WNDW_DEBUG, WNDW_LIDAR, WNDW_CAMERA };
constexpr size_t  WNDW_WIDTHS[] = { 700, 400, 200 };
constexpr size_t  WNDW_ORIGIN[] = { 100, 50 };
constexpr auto    WNDW_MARGIN   = 20;

// error messages

constexpr auto    ERR_NOT_INIT           = "System is not initialized.";
constexpr auto    ERR_RE_INIT            = "System is already initialized.";
constexpr auto    ERR_FL_ENGINE_NOTRDY   = "Fuzzylite engine is not ready:n";
constexpr auto    ERR_VELCMD_NAN         = "Velocity data is NaN.";

// scale factor floor_plan.png image

constexpr auto    FLOOR_PLAN_SCALE       = ( ( 120 / 84.67 ) + ( 80 / 56.44 ) ) / 2.0;