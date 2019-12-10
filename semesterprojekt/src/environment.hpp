#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <random>
#include <algorithm>
#include <map>

#define STOP 100
#define SWEEP_REWARD 1.00f

#define TEST 1

#define WALL_TYPE '-'
#define WALKABLE_TYPE ' '
#define MARBLE_TYPE 'X'
#define UNDEFINED_TYPE '*'


struct state
{

public:
    state(std::initializer_list<double> Q_values): Q_value{Q_values}
    {};
    // State_action_values ie.  {Q_value[0] -> UP, Q_value[1] -> RIGHT, Q_value[2] -> DOWN, Q_value[3] -> LEFT} if the state has all 4 actions avaible.
    std::vector<double> Q_value;
};

typedef std::vector<std::vector<state>> environment_map_t;
typedef std::vector<std::vector<char>> environment_type_map_t;
typedef std::vector<int> environment_key_t;

class environment
{
public:

    environment(double transition_penalty, double wall_colision_penalty, double goal_reward, cv::Mat& environment, double optimistic_init=0.0);

    ~environment();

    double
    get_reward(const cv::Point& state_pos);

    state&
    get_state(const cv::Point& state_pos);

    char
    get_state_type(const cv::Point& state_pos);

    environment_map_t&
    pick_up_marble(const cv::Point& marble_positon);

    environment_map_t&
    get_environment(const environment_key_t& remaining_marbles);

    const environment_key_t&
    get_current_remaining_marbles();

    int
    add_marble(const cv::Point& pos);

    environment_map_t&
    init_environment();

    void
    print_environment_type_map();

    void
    print_environment_Q_optimal(environment_map_t& environment);

    cv::Mat
    get_environment_img(bool show_marbles=true);

    int64_t
    get_total_environments_discovered();

    void
    reset_all_environments();

    const double transition_penalty;
    const double wall_colision_penalty;
    const double goal_reward;

private:
    cv::Mat environment_img;
    cv::Size environment_size;
    environment_key_t initial_marble_locations;
    std::map<environment_key_t, environment_map_t> environments_cache;
    environment_type_map_t type_map;
    environment_key_t current_remaining_marbles;
    environment_map_t* current_environment;
    double optimistic_init;
    void
    init_type_map();

    int
    get_marble_key(const cv::Point& marble_pos);

    char
    get_state_type(uchar pixelvalue);

    void
    insert_marble_to_type_map(const cv::Point& marble_pos);

    environment_map_t&
    generate_fresh_environment();

    std::vector<cv::Point> get_marble_point_position();
};





/*
struct map_hasher
{
    int operator()(const std::vector<int>& V) const
    {
        int hash = 0;
        for (int i = 0; i < V.size(); i++)
            hash += V[i]; // Can be anything
        return hash << V.size();
    }
};

*/



/*



state
environment::get_default_state(int x, int y)
{
    return std::move( state{{x,y}, {action::UP, action::RIGHT, action::DOWN, action::LEFT}, {0.0, 0.0, 0.0, 0.0}, reward, false});
}

state
environment::get_wall_state(int x, int y)
{
    return std::move( state{ {x,y}, {}, {0.0}, wall_reward, false});
}


bool
    add_to_cache(std::vector<int>& marble_locations_remaining);

    bool
environment::add_to_cache(std::vector<int>& marble_locations_remaining)
{
    std::sort(marble_locations_remaining.begin(),marble_locations_remaining.end());
    // TODO generate a map and insert into environments_cache.
    //return environments_cache.insert(marble_locations_remaining);
    return false;
}

*/