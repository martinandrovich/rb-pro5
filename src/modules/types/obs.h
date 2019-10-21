# pragma once

#include <ostream>
#include <map>

#include <gazebo/msgs/msgs.hh>
#include <opencv2/opencv.hpp>

#include "pose.h"

// --------------------------------------------------------------------------------
// declarations for ::obs_t
// --------------------------------------------------------------------------------

struct obs_t
{
	obs_t();
	obs_t(float dir, float dist, pos_t pos);

	float dir;
	float dist;
	pos_t pos;

	bool
	operator == (const obs_t& other) const;

	friend std::ostream&
	operator << (std::ostream& out, const obs_t& obj);
};

// --------------------------------------------------------------------------------
// redefinitions
// --------------------------------------------------------------------------------

using obs_list_t = std::map<std::string, obs_t>;

// --------------------------------------------------------------------------------
// definitions for ::obs_t
// --------------------------------------------------------------------------------

inline
obs_t::obs_t() : dir(INFINITY), dist(INFINITY), pos(pos_t{ 0, 0, 0 }) {}

inline
obs_t::obs_t(float dir, float dist, pos_t pos) : dir(dir), dist(dist), pos(pos) {}

inline bool
obs_t::operator == (const obs_t& other) const
{
	return (this->pos == other.pos);
}

inline std::ostream&
operator << (std::ostream& out, const obs_t& obj)
{
	return out << "a: " << obj.dir << " | d: " << obj.dist << " @ " << obj.pos;
}