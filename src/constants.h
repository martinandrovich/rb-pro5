#pragma once

#include <string>
#include <stdexcept>
#include <chrono>
#include <initializer_list>

#define constexpr_string inline const std::string

using ilf = std::initializer_list<float>;

// constants

constexpr auto    EPSILON                     = 1e-6; // used for float comparisons
constexpr auto    INV_SQRT_2PI                = 0.3989422804014327;

// paths

constexpr_string  PATH_ROOT                   = "";
constexpr_string  PATH_ASSETS                 = PATH_ROOT + "assets/data/";
constexpr_string  PATH_IMG_ENVIRON_DEMO       = PATH_ROOT + "assets/img/environ-01.png";
constexpr_string  PATH_IMG_GVD_MAP            = PATH_ROOT + "assets/img/gvd-map.png";
constexpr_string  PATH_IMG_GVD_OUTPUT         = PATH_ROOT + "assets/gvd-output/";
constexpr_string  PATH_IMG_GRAD_TEST          = PATH_ROOT + "assets/img/grad-test-3.png";
constexpr_string  PATH_IMG_SMALLWORLD         = PATH_ROOT + "gazebo/models/smallworld/meshes/floor_plan.png";
constexpr_string  PATH_IMG_BIGWORLD           = PATH_ROOT + "gazebo/models/bigworld/meshes/floor_plan.png";
constexpr_string  PATH_IMG_BIGWORLD_ABSTRACT  = PATH_ROOT + "assets/img/floor_plan_abstract.png";
constexpr_string  PATH_FLOOR_PLAN             = PATH_ROOT + "assets/img/floor_plan.png";
constexpr_string  PATH_FUZZY_OBS_AVOID        = PATH_ROOT + "assets/data/fuzzy-obs-avoid.fll";
constexpr_string  PATH_FUZZY_SIMPLE_NAVIGATOR = PATH_ROOT + "assets/data/simpleNavigator.fll";
constexpr_string  PATH_FONT_CONSOLAS          = PATH_ROOT + "assets/data/consolas.ttf";

// system config

constexpr auto    RUN_FREQ_MS                 = std::chrono::milliseconds(20); // ms
constexpr auto    MAX_DIST_TO_OBSTACLE        = 0.7f; // meters
constexpr ilf     GOAL_POS                    = { 0.f, 0.f, 0.f }; // meters
constexpr auto    FUZZY_SCALING_FACTOR        = 1.0f;
constexpr auto    SCALE_METER_PER_PX          = 0.1f; // meters per pixel
constexpr auto    FLOOR_PLAN_SCALE            = ((120 / 84.67) + (80 / 56.44)) / 2.0; // scale factor floor_plan.png image
constexpr ilf     DIM_SMALLWORLD              = { 14.11f, 10.58f };
constexpr ilf     DIM_BIGWORLD                = { 84.66f, 56.44f };
constexpr auto    GVD_VERTEX_RADIUS           = SCALE_METER_PER_PX * 40;
constexpr auto    LIDAR_MAX_RANGE             = 200;
constexpr auto    IMG_BORDER_SIZE             = 1 ; // width in pixels

constexpr auto    TEST_GVD                    = false;
constexpr auto    TEST_LINE_SEG               = false;
constexpr auto    TEST_MARBLE_DETECT          = true;
constexpr auto    TEST_EXIT_AFTER             = true;

constexpr auto    USE_OBS_AVOID               = true;
constexpr auto    USE_PARTICLE_FILTER         = false;
constexpr auto    USE_LOCALIZATION            = false;

constexpr auto    PTCLFILT_PARTICLES          = 2500;
constexpr auto    PTCLFILT_IMG_SCALE          = 10;
constexpr auto    PTCLFILT_LIDAR_SCALE        = 10 * PTCLFILT_IMG_SCALE;
constexpr auto    PTCLFILT_ANGLES             = 276;
constexpr auto    PTCLFILT_START_VAL          = 3.1347;
constexpr auto    PTCLFILT_DELTA              = 2.04086 - 2.01806;
constexpr auto    PTCLFILT_THREADS            = 16;
constexpr auto    PTCLFILT_POS_NOISE_SIGMA    = 1.8;
constexpr auto    PTCLFILT_ANG_NOISE_SIGMA    = 0.1;

// windows

constexpr auto    WNDW_DEBUG                  = "debug";
constexpr auto    WNDW_LIDAR                  = "lidar";
constexpr auto    WNDW_CAMERA                 = "camera";
constexpr auto    WNDW_PTCLFILT               = "particle filter";
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
constexpr auto    ERR_NO_IMPL                 = "No implementation.";
constexpr auto    ERR_NUM_NOT_POS             = "Number must be positive.";
constexpr auto    ERR_EXCEED_LIDAR_RANGE      = "Number of lidar rays exceed maximum; check model.sdf and cached model of lidar sensor.";
constexpr auto    ERR_PATTERN_MISMATCH_3x3    = "Pattern is not of size 3x3.";
constexpr auto    ERR_FIX_ASYM_V_SHAPE        = "Unexpected error when fixing asymmetric v-shapes; undefined fixer pattern.";