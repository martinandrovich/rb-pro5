#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <random>
#include <algorithm>
#include <unistd.h>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <fstream>

#if 0
/*
#define STOP 100
#define SWEEP_REWARD 1.00f

#define TEST 0


enum actions
{
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    SWEEP = 4,
};

struct state
{
public:
    // Set position
    cv::Point pos;

    // Set valid actions
    std::vector<actions> actions_valid;

    // Set state actions pairs
    std::vector<double> Q_value;

    // Reward
    double reward;

    // Visited
    bool sweeped;

};

/*
marple_map = map
for y in range(100):
    for x in range(120):
        if marple(x, y):
            marple_map.insert(y*120 + x)


https://stackoverflow.com/questions/2634690/good-hash-function-for-a-2d-index 

a map is represented by the number of marples and their location
    vector<int> environment;

Example 3x3 points maps: 1 is maple, 0 is blank

 A   B   C
010 011 010
100 100 000
001 001 111

representation:

A1: [1, 3, 8]       ([1, 3], [1, 8], [3, 8])
A2: [1, 8, 3]
B: [1, 2, 3, 8]
C: [1, 6, 7, 8]

Example 2: 10x10 points map

     D
0000000000
0000000000
0000000000
0000000000
0000000000
0000000000
0000000000
0000000000
0000000000
0000000001

D: [98]

Known maps could be described in a simple C++ map

std::map<std::vector<int>, environment, hash_function> stored_environments;



Construct vector of all generated marbles
std::vector<cv::point>>

get_marble_hash_idx


*/


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



class map
{
public:

    map(double reward_, cv::Size map_size_);
    map(double reward_, cv::Size map_size_, cv::Mat& environment);

    ~map();

    bool
    find_map_t_ptr(const std::vector<int> &trail);

    std::vector<std::vector<state>>*
    find_map(const std::vector<int> &trail);

    void
    update_end_map();

    void
    create_maps();

    void
    create_one_map(const std::vector<int> &trail);

    std::vector<actions>
    actions_possible(const cv::Point &cur, const bool &sweeped);

    void
    print_map(std::vector<int> &trail);

    void
    print_all_maps();

    void
    action_on_map(std::vector<int> &trail, cv::Point &pos, actions &action, double &cur_reward, double &cmf_distance);

    std::pair<actions, double>
    get_optimal_action(std::vector<int> &trail, cv::Point &cur);

    std::pair<actions, double>
    get_random_action(std::vector<int> &trail, cv::Point &cur);

    void
    update_q_value(std::vector<int> &trail, double &cur_reward, actions &action, cv::Point &pos, double &Q_value_next, double &learning_rate, double &discount_val);


/*_______________________________________________________*/

    int
    add_marble_to_map(const cv::Point& pos);

    void
    generate_environment();

    void 
    print_initial_environment();
 

private:
    std::unordered_map<std::vector<int>, std::shared_ptr<std::vector<std::vector<state>>>, map_hasher> cache;
    std::uniform_real_distribution<double> marbles_in_each_area;

    cv::Size map_size;
    double reward;
    cv::Mat img;
    std::vector<std::vector<double>> marbles_in_pos;
    cv::Point recharge_station;
    std::vector<std::vector<int>> trails;

    std::vector<int> known_marbles;
    cv::Size environment_size;
    std::map<std::vector<int>, std::vector<std::vector<state>> > environments_cache;
    double wall_reward;
    std::vector<std::vector<state>> initial_environment;
    cv::Mat environment;
    
    bool
    add_to_cache(std::vector<int>& marble_locations_remaining);

    state
    get_default_state(int x, int y);

    state
    get_wall_state(int x, int y);
};







#endif