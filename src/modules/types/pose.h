#pragma once

#include <iostream>
#include <atomic>
#include <mutex>

#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Quaternion.hh>
#include "../debug.h"

// --------------------------------------------------------------------------------
// declarations for ::pos_t
// --------------------------------------------------------------------------------

struct pos_t 
{
	float x = 0.f;
	float y = 0.f;
	float z = 0.f;

	template<typename T> auto
	get(const T* var);

	bool
	operator == (const pos_t& other) const;

	friend std::ostream&
	operator << (std::ostream& out, const pos_t& obj);
};

// --------------------------------------------------------------------------------
// declarations for ::orient_t
// --------------------------------------------------------------------------------

struct orient_t
{
public:

	float w = 0.f;
	float x = 0.f;
	float y = 0.f;
	float z = 0.f;

	float roll  = 0.f;
	float pitch = 0.f;
	float yaw   = 0.f;

	void
	set(const gazebo::msgs::Quaternion& orient_msg);

	template<typename T> auto
	get(const T* var);

	ignition::math::Quaternion<float>
	get_quaternion();

	bool
	operator == (const orient_t& other) const;

	friend std::ostream&
	operator << (std::ostream& out, orient_t& obj);

private:

	std::mutex mutex;
	
};

// --------------------------------------------------------------------------------
// declarations for ::vel_t
// --------------------------------------------------------------------------------

struct vel_t
{
	float trans = 0.f;
	float ang   = 0.f;

	ignition::math::Pose3d
	get_pose();

	friend std::ostream&
	operator << (std::ostream& out, const vel_t& obj);
};

// --------------------------------------------------------------------------------
// declarations for ::pose_t
// --------------------------------------------------------------------------------

class pose_t
{	
public:

	pos_t pos;
	orient_t orient;

	void
	set(const gazebo::msgs::Pose& pose);

	pos_t
	get_pos();

	ignition::math::Quaternion<float>
	get_quaternion();

	float
	dir(const pos_t& other);

	float
	dist(const pos_t& other);

	float
	abs_dist();

	friend std::ostream&
	operator << (std::ostream& out, const pose_t& obj)
	{
		return out
			<< std::setprecision(2) << std::fixed << std::setw(6)
			<< obj.pos.x << std::setw(6)
			<< obj.pos.y << std::setw(6)
			<< obj.pos.z << std::setw(6)
			<< obj.orient.w << std::setw(6)
			<< obj.orient.x << std::setw(6)
			<< obj.orient.y << std::setw(6)
			<< obj.orient.z << std::endl;
	}

	
private:

	std::mutex mutex;

};

// --------------------------------------------------------------------------------
// definitions for ::pos_t, ::orient_t, ::vel_t, ::pose_t
// --------------------------------------------------------------------------------

// -- pos_t -----------------------------------------------------------------------

inline bool
pos_t::operator == (const pos_t& other) const
{
	return (this->x == other.x && this->y == other.y && this->z == other.z);
}

inline std::ostream&
operator << (std::ostream& out, const pos_t& obj)
{
	return out << "x: " << obj.x << " | y: " << obj.y << " | z: " << obj.z;
}

// -- orient_t --------------------------------------------------------------------

inline void
orient_t::set(const gazebo::msgs::Quaternion& orient_msg)
{
	this->mutex.lock(); // CRITICAL SECTION BEGIN
	
	// set values
	this->w = orient_msg.w();
	this->x = orient_msg.x();
	this->y = orient_msg.y();
	this->z = orient_msg.z();

	// calculate roll, pitch and yaw
	auto q = ignition::math::Quaternion<float>(this->w, this->x, this->y, this->z);

	this->roll  = q.Roll();
	this->pitch = q.Pitch();
	this->yaw   = q.Yaw();

	this->mutex.unlock(); // CRITICAL SECTION END
}

template<typename T>
auto orient_t::get(const T* var)
{
	std::lock_guard<std::mutex> lock(this->mutex);
	return *var;
}

inline ignition::math::Quaternion<float>
orient_t::get_quaternion()
{
	std::lock_guard<std::mutex> lock(this->mutex);
	return ignition::math::Quaternion<float>(this->w, this->x, this->y, this->z);
}

inline bool
orient_t::operator == (const orient_t& other) const
{
	return (this->x == other.x && this->y == other.y && this->z == other.z);
}

inline std::ostream&
operator << (std::ostream& out, orient_t& obj)
{
	std::lock_guard<std::mutex> lock(obj.mutex);
	return out << "w: " << obj.w << " | x: " << obj.x << " | y: " << obj.y << " | z: " << obj.z;
}

// -- vel_t -----------------------------------------------------------------------

inline ignition::math::Pose3d
vel_t::get_pose()
{
	return ignition::math::Pose3d(this->trans, 0, 0, 0, 0, this->ang);
}

inline std::ostream&
operator << (std::ostream& out, const vel_t& obj)
{
	return out << "x: " << obj.trans << " | w: " << obj.ang;
}

// -- pose_t ----------------------------------------------------------------------

inline void
pose_t::set(const gazebo::msgs::Pose& pose)
{
	this->mutex.lock(); // CRITICAL SECTION BEGIN

	// set position
	this->pos.x = pose.position().x();
	this->pos.y = pose.position().y();
	this->pos.z = pose.position().z();

	// set orientation
	this->orient.set(pose.orientation());

	this->mutex.unlock(); // CRITICAL SECTION END
}

inline pos_t
pose_t::get_pos()
{
	std::lock_guard<std::mutex> lock(this->mutex);
	return this->pos;
}

inline ignition::math::Quaternion<float>
pose_t::get_quaternion()
{
	std::lock_guard<std::mutex> lock(this->mutex);
	return ignition::math::Quaternion<float>(this->orient.w, this->orient.x, this->orient.y, this->orient.z);
}

inline float
pose_t::dir(const pos_t& other)
{
	
	this->mutex.lock(); // CRITICAL SECTION BEGIN

	float theta_goal = atan2(other.y - this->pos.y, other.x - this->pos.x);
	float theta_self = this->orient.yaw; 	

	this->mutex.unlock(); // CRITICAL SECTION END
	
	// calculate delta in theta with direction
	// negative = ccw
	float theta_diff = theta_goal - theta_self;

	float theta_diff_org = theta_diff;
	float theta_goal_org = theta_goal;
	float theta_self_org = theta_self;

	// theta difference must be relative to the gazebo frame and not exceed PI
	// with 0 to PI in top plane and 0 to -PI in bottom plane
	// e.g. if target is 3/4*PI and self is -3/4 PI, the result must be 
	// diff = -3/4*PI - 3/4*PI = 6/4*PI (wrong) -> -1/2*PI (correct)

	// modulate theta keeping signedness
	auto theta_diff2 = fmod(theta_diff, (theta_diff > 0) ? M_PI : -M_PI);

	if(theta_diff > M_PI)
	{
		theta_self += 2 * M_PI;
	}
	else if(theta_diff < -M_PI)
	{
		theta_goal += 2 * M_PI;
	}

	theta_diff = theta_goal - theta_self;

	debug::cout << "goal: " << theta_goal_org << " | self: " << theta_self_org << " | original: " << theta_diff_org << " | ifs: " << theta_diff << " | mod: " << theta_diff2 << "\n";
	
	return theta_diff;
}

inline float
pose_t::dist(const pos_t& other)
{
	std::lock_guard<std::mutex> lock(this->mutex); 	 

	float dist = sqrt(pow(other.y - this->pos.y, 2) + pow(other.x - this->pos.x, 2));			

	return dist;
}
