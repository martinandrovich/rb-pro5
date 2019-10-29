#pragma once

#include <fl/Headers.h>

#include "../constants.h"
#include "types/pose.h"
#include "types/obs.h"
#include <cmath>

// --------------------------------------------------------------------------------
// declarations for ::flctrl
// --------------------------------------------------------------------------------

namespace flctrl
{
	void
	goal_nav(pose_t& pose, pos_t& goal, vel_t& vel_cmd);

	void
	obs_avoid(obs_list_t& obs_list, vel_t& vel_cmd);

	void
	obs_avoid_simple(obs_list_t& obs_list, vel_t& vel_cmd);
	
	static float 
	_get_val(fl::OutputVariable* var);
}

// --------------------------------------------------------------------------------
// method defintions for ::flctrl
// --------------------------------------------------------------------------------

inline void
flctrl::goal_nav(pose_t& pose, pos_t& goal, vel_t& vel_cmd)
{
	// load engine and check for readiness
	static fl::Engine* engine = fl::FllImporter().fromFile(PATH_FUZZY_SIMPLE_NAVIGATOR);

	if (std::string status; not engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);
	
	static fl::InputVariable* goal_dist    = engine->getInputVariable("goal_dist");
	static fl::InputVariable* goal_dir 	   = engine->getInputVariable("goal_dir");
	static fl::OutputVariable* robot_dir   = engine->getOutputVariable("ang_vel");
	static fl::OutputVariable* robot_speed = engine->getOutputVariable("robot_speed");	

	// get inputs
	float dir  = pose.dir(goal);
	float dist = pose.dist(goal);


	// feed input to the fl engine
	goal_dir->setValue(dir);
	goal_dist->setValue(dist);

	// process inputs
	engine->process();
	
	// extract outputs
	vel_cmd.trans = FUZZY_SCALING_FACTOR * _get_val(robot_speed);
	vel_cmd.ang   = FUZZY_SCALING_FACTOR * _get_val(robot_dir);	
}

inline void
flctrl::obs_avoid(obs_list_t& obs_list, vel_t& vel_cmd)
{
	// load engine and check for readiness
	static fl::Engine* engine = fl::FllImporter().fromFile(PATH_FUZZY_OBS_AVOID);

	if (std::string status; not engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);

	// variables (loaded once)
	static fl::InputVariable*  obs_c_dist = engine->getInputVariable("forward");
	static fl::InputVariable*  obs_r_dist = engine->getInputVariable("right");
	static fl::InputVariable*  obs_l_dist = engine->getInputVariable("left");
	static fl::OutputVariable* rob_vel    = engine->getOutputVariable("velocity");
	static fl::OutputVariable* rob_angvel = engine->getOutputVariable("omega");

	// apply inputs
	obs_c_dist->setValue(obs_list["center"].dist);
	obs_r_dist->setValue(obs_list["right"].dist);
	obs_l_dist->setValue(obs_list["left"].dist);

	// process data
	engine->process();

	// export outputs
	vel_cmd.trans = FUZZY_SCALING_FACTOR * (float)rob_vel->getValue();
	vel_cmd.ang   = FUZZY_SCALING_FACTOR * (float)rob_angvel->getValue();
}

inline void
flctrl::obs_avoid_simple(obs_list_t& obs_list, vel_t& vel_cmd)
{
	// load engine and check for readiness
	static fl::Engine* engine = fl::FllImporter().fromFile("assets/data/fuzzy-obs-avoid-simple.fll");

	if (std::string status; not engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);

	// variables (loaded once)
	static fl::InputVariable*  obs_dir    = engine->getInputVariable("obs_dir");
	static fl::InputVariable*  obs_dist   = engine->getInputVariable("obs_dist");
	static fl::OutputVariable* rob_vel    = engine->getOutputVariable("rob_vel");
	static fl::OutputVariable* rob_angvel = engine->getOutputVariable("rob_angvel");

	// apply inputs
	obs_dir->setValue(obs_list["any"].dir);
	obs_dist->setValue(obs_list["any"].dist);

	// process data
	engine->process();

	// export outputs
	vel_cmd.trans = FUZZY_SCALING_FACTOR * (float)rob_vel->getValue();
	vel_cmd.ang   = FUZZY_SCALING_FACTOR * (float)rob_angvel->getValue();
}

static float 
flctrl::_get_val(fl::OutputVariable* var)
{
	auto tmp = var->getValue();
	if(std::isnan(tmp))
	{
		debug::dout << "#################### ERROR OCCURED #################### \n";		
		debug::cout << "#################### ERROR OCCURED #################### \n";
	}
	return tmp;
}
