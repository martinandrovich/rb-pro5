#include "environment.hpp"

environment::environment(double transition_penalty, double wall_colision_penalty, double goal_reward, cv::Mat& img, double optimistic_init)
: transition_penalty{transition_penalty}
, wall_colision_penalty{wall_colision_penalty}
, goal_reward{goal_reward}
, environment_img(img.clone())
, environment_size{environment_img.size()}
, initial_marble_locations()
, environments_cache()
, type_map()
, current_remaining_marbles()
, current_environment{nullptr}
, optimistic_init{optimistic_init}
{
    init_type_map();
    std::cout << "Environment constructed. " << std::endl;
}

environment::~environment()
{}

double
environment::get_reward(const cv::Point& state_pos)
{
    char state_type = get_state_type(state_pos);
    switch (state_type)
    {
    case WALL_TYPE:
        return wall_colision_penalty;

    case WALKABLE_TYPE:
        return transition_penalty;

    case MARBLE_TYPE:
        return goal_reward;

    default:
        break;
    }
    return transition_penalty;
}

state&
environment::get_state(const cv::Point& state_pos)
{
    return(*current_environment)[state_pos.x][state_pos.y];
    //return environments_cache[current_remaining_marbles][state_pos.x][state_pos.y];
}

char
environment::get_state_type(const cv::Point& state_pos)
{
    if(state_pos.x < environment_size.width && state_pos.x >= 0 && state_pos.y < environment_size.height && state_pos.y >= 0)
    {
        auto state_type = type_map[state_pos.x][state_pos.y];

        if(state_type == MARBLE_TYPE)
        {
            //Check if the marble has been picked up.
            auto marble_key = get_marble_key(state_pos);
            if(std::find(current_remaining_marbles.begin(), current_remaining_marbles.end(), marble_key) != current_remaining_marbles.end())
            {
                return state_type;
            }
            else
            {
                return WALKABLE_TYPE;
            }
        }
        return state_type;
    }
    else return WALL_TYPE;
}

environment_map_t&
environment::pick_up_marble(const cv::Point& marble_positon)
{
    int marble_key = get_marble_key(marble_positon);
    current_remaining_marbles.erase(
        std::remove(current_remaining_marbles.begin(), current_remaining_marbles.end(), marble_key)
        , current_remaining_marbles.end());

    std::sort(current_remaining_marbles.begin(),current_remaining_marbles.end());
    auto it = environments_cache.find(current_remaining_marbles);
    if(it != environments_cache.end())
    {
        current_environment = &(*it).second;
        return it->second;
    }
    else
    {
        environment_map_t& new_map = generate_fresh_environment();
        current_environment = &new_map;
        return new_map;
    }
}

environment_map_t&
environment::get_environment(const environment_key_t& remaining_marbles)
{
    if(environments_cache.find(remaining_marbles) != environments_cache.end())
    {
        return environments_cache[remaining_marbles];
    }
    else
    {
        return environments_cache[initial_marble_locations];
    }
}

const environment_key_t&
environment::get_current_remaining_marbles()
{
    return current_remaining_marbles;
}

int
environment::add_marble(const cv::Point& pos)
{
    int vectorized_positon = get_marble_key(pos);
    current_remaining_marbles.push_back(vectorized_positon);
    initial_marble_locations.push_back(vectorized_positon);
    std::sort(current_remaining_marbles.begin(),current_remaining_marbles.end());
    std::sort(initial_marble_locations.begin(),initial_marble_locations.end());
    insert_marble_to_type_map(pos);
    return vectorized_positon;
}

environment_map_t&
environment::init_environment()
{
    assert(initial_marble_locations.size() >= 1);
    current_remaining_marbles = initial_marble_locations;
    if( environments_cache.find(initial_marble_locations) != environments_cache.end())
    {
        current_environment = &environments_cache[initial_marble_locations];
        return environments_cache[initial_marble_locations];
    }
    else
    {
        environment_map_t& new_map = generate_fresh_environment();
        current_environment = &new_map;
        return new_map;
    }
}

int
environment::get_marble_key(const cv::Point& marble_pos)
{
    return marble_pos.x + marble_pos.y * environment_size.width;
}

void
environment::init_type_map()
{
    type_map.reserve(environment_img.cols);
    for(int x = 0; x < environment_img.cols; x++)
    {
        type_map.push_back(std::vector<char>());
        for(int y = 0; y < environment_img.rows; y++)
        {
            uchar pixel = environment_img.at<uchar>(y, x);
            type_map[x].push_back(get_state_type(pixel));
        }
    }
}

void
environment::print_environment_Q_optimal(environment_map_t& environment)
{
    std::cout << "Printing the optimal path that was chosen by the agent." << std::endl;

     for(int y= 0; y < environment_size.height; y++)
    {
        for(const auto& x : environment)
        {
            std::cout << std::setw(8) << std::setprecision(2) << *std::max_element(x[y].Q_value.begin(),x[y].Q_value.end());
        }
        std::cout << std::endl;
    }
}

cv::Mat
environment::get_environment_img(bool show_marbles)
{
    cv::Mat tmp = environment_img.clone();
    if(show_marbles)
    {
        cv::Mat copy;
        cv::cvtColor(tmp, copy,cv::COLOR_GRAY2BGR);
        for(const auto& pos : get_marble_point_position())
        {
            cv::circle(copy, pos, 1, cv::Scalar(255, 0, 0), 1);
        }
        return copy;
    }
    return tmp;
}

int64_t
environment::get_total_environments_discovered()
{
    return environments_cache.size();
}

void
environment::reset_all_environments()
{
    environments_cache.clear();
}

void
environment::print_environment_type_map()
{
    std::cout << "Printing Environment type map : { ["
    << WALL_TYPE << "] = wall; ["
    << WALKABLE_TYPE << "] = passable_state; ["
    << UNDEFINED_TYPE << "] = undefined; ["
    << MARBLE_TYPE << "] = marble_type} " << std::endl;
    for(int y= 0; y < environment_size.height; y++)
    {
        for(const auto& x : type_map)
        {
            std::cout << std::setw(4) << x[y];
        }
        std::cout << std::endl;
    }
}

char
environment::get_state_type(uchar pixelvalue)
{
    switch(pixelvalue)
    {
    case 0:
        return WALL_TYPE;
        break;

    case 255:
        return WALKABLE_TYPE;

    default:
        return UNDEFINED_TYPE; // This should never happen..
        break;
    }
    return UNDEFINED_TYPE;
}

void
environment::insert_marble_to_type_map(const cv::Point& marble_pos)
{
    type_map[marble_pos.x][marble_pos.y] = MARBLE_TYPE;
}

environment_map_t&
environment::generate_fresh_environment()
{
    environment_map_t new_map(environment_size.width, std::vector<state>(environment_size.height, {optimistic_init, optimistic_init, optimistic_init, optimistic_init}));
    environments_cache[current_remaining_marbles] = new_map;
    return environments_cache[current_remaining_marbles];
}

std::vector<cv::Point>
environment::get_marble_point_position()
{
    std::vector<cv::Point> marble_points;
    for(const auto& oneD_position : initial_marble_locations)
    {
        marble_points.emplace_back(oneD_position % environment_size.width, oneD_position/environment_size.width);
    }
    return marble_points;
}