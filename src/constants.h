#pragma once

#include <string>
#include <stdexcept>
#include <chrono>
#include <initializer_list>

#define constexpr_string inline const std::string

using ilf = std::initializer_list<float>;

// paths

constexpr_string  PATH_ROOT = "";
constexpr_string  PATH_ASSETS                 = PATH_ROOT + "assets/data/";
constexpr_string  PATH_IMG_ENVIRON_DEMO       = PATH_ROOT + "assets/img/environ-01.png";
constexpr_string  PATH_IMG_SMALLWORLD         = PATH_ROOT + "gazebo/models/smallworld/meshes/floor_plan.png";
constexpr_string  PATH_IMG_BIGWORLD           = PATH_ROOT + "gazebo/models/bigworld/meshes/floor_plan.png";
constexpr_string  PATH_FUZZY_OBS_AVOID        = PATH_ROOT + "assets/data/fuzzy-obs-avoid.fll";
constexpr_string  PATH_FUZZY_SIMPLE_NAVIGATOR = PATH_ROOT + "assets/data/simpleNavigator.fll";
constexpr_string  PATH_FONT_CONSOLAS          = PATH_ROOT + "assets/data/consolas.ttf";

// system config

constexpr auto    RUN_FREQ_MS                 = std::chrono::milliseconds(10); // ms
constexpr auto    MAX_DIST_TO_OBSTACLE        = 0.7f; // meters
constexpr ilf     GOAL_POS                    = { 4.f, -3.f, 0.f }; // meters
constexpr auto    LIDAR_RANGE_LIMIT           = 10; // number of rays
constexpr auto    FUZZY_SCALING_FACTOR        = 1.0f;
constexpr auto    SCALE_METER_PER_PX          = 0.1f; // meters per pixel
constexpr ilf     DIM_SMALLWORLD              = { 14.11f, 10.58f };
constexpr ilf     DIM_BIGWORLD                = { 84.66f, 56.44f };

constexpr auto    TEST_CELL_DECOMP            = true;
constexpr auto    TEST_MARBLE_DETECT          = true;
constexpr auto    USE_LOCALIZATION            = true;
constexpr auto    USE_OBS_AVOID               = true;

// windows

constexpr auto    WNDW_CAMERA                 = "camera";
constexpr auto    WNDW_LIDAR                  = "lidar";
constexpr auto    WNDW_DEBUG                  = "debug";
constexpr auto	  WNDW_HANDLES                = { WNDW_DEBUG, WNDW_LIDAR, WNDW_CAMERA };
constexpr size_t  WNDW_WIDTHS[]               = { 700, 400, 200 };
constexpr size_t  WNDW_ORIGIN[]               = { 100, 50 };
constexpr auto    WNDW_MARGIN                 = 20;

// error messages

constexpr auto    ERR_NOT_INIT                = "System is not initialized.";
constexpr auto    ERR_RE_INIT                 = "System is already initialized.";
constexpr auto    ERR_FL_ENGINE_NOTRDY        = "Fuzzylite engine is not ready:n";
constexpr auto    ERR_VELCMD_NAN              = "Velocity data is NaN.";
constexpr auto    ERR_IMG_EMPTY               = "The image is empty.";
constexpr auto    ERR_IMG_NOT_GRAY            = "The image is not grayscale.";