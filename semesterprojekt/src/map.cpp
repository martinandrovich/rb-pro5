#include "map.hpp"

#if 0
map::map(double reward_, cv::Size map_size_)
: map_size(map_size_)
, marbles_in_each_area(0, 1)
, reward(reward_)
, cache()
, img(map_size, CV_8UC1)
, known_marbles()

{
    std::cout << "Init map... \n";

    // Create Vector Maps
    //create_maps();

    //Update end map;
    update_end_map();

    // Create a map from openCV
    img = cv::Mat::zeros(map_size, CV_8UC1);

    std::cout << "Map is created... " << std::endl;
}


map::map(double reward_, cv::Size map_size_, cv::Mat& environment)
: map_size(map_size_)
, marbles_in_each_area(0, 1)
, reward(reward_)
, cache()
, img(map_size, CV_8UC1)
, known_marbles()
, environment_size(environment.size())
, environment(environment.clone())
{
  std::cout << "Map constructed. " << std::endl;
}

map::~map(){};

bool
map::find_map_t_ptr(const std::vector<int> &trail)
/*

{
    // try to find trail
    auto got = cache.find(trail);

    // if not possible
    if (got == cache.end())
        return false;
    else
        while (got != cache.end())
        {
            if (got->first == trail)
            {
                return true;
            }
            got++;
        }
}

std::vector<std::vector<state>>*
map::find_map(const std::vector<int> &trail)
/*
* Look up the map, by using current trail Remember the trail consists of more information
*/
{

    auto got = cache.find(trail);

    //std::cout << cache.size() << std::endl;

    while (got != cache.end())
    {
        if (got->first == trail)
        {

            return (got->second.get());
        }
        got++;
    }

    create_one_map(trail);

    return find_map(trail);
}

void
map::update_end_map()
{
    std::vector<int> new_trail;
    for (size_t i = 0; i < map_size.width * map_size.height; i++)
        new_trail.push_back(i + 1);
    std::vector<std::vector<state>> *ptr_map = find_map(new_trail);
    for (auto &row : (*ptr_map))
        for (auto &col : row)
            for (auto &Q_val : col.Q_value)
                Q_val = STOP;
}

void
map::create_maps()
/*
*
*/
{
    int trail_start_idx = 0, trail_end_idx = 1, iterate_val = 0;
    auto trail = std::vector<int>(map_size.height * map_size.width, 0);
    create_one_map(trail);
    while (trail_start_idx != trail_end_idx)
    {
        iterate_val++;
        std::vector<int> trial_new;
        for (size_t i = trail_start_idx; i < trail_end_idx; i++)
        {
            trial_new = trails[i];
            for (size_t j = 0; j < trails[0].size(); j++)
            {
                if (trial_new[j] != j + 1)
                {
                    trial_new[j] = j + 1;
                    if (!find_map_t_ptr(trial_new))
                    {
                        create_one_map(trial_new);
                    }
                    trial_new[j] -= j + 1;
                }
            }
        }

        trail_start_idx = trail_end_idx;
        trail_end_idx = trails.size();
    }
    //trails.clear();
}

void
map::create_one_map(const std::vector<int> &trail)
/*
* Create a map, everything is initialised
*/
{
    trails.push_back(trail);
    auto map_t = std::vector<std::vector<state>>(map_size.height, std::vector<state>(map_size.width, {{0, 0}, {UP}, {0}, reward, false}));

    for (size_t i = 0; i < map_size.height; i++)
        for (size_t j = 0; j < map_size.width; j++)
        {
            map_t[i][j].pos = cv::Point(i, j);
            map_t[i][j].sweeped = (trail[map_size.height * i + j] != 0) ? 0 : 1;
            map_t[i][j].actions_valid = actions_possible(map_t[i][j].pos, map_t[i][j].sweeped);
            map_t[i][j].Q_value = std::vector<double>(map_t[i][j].actions_valid.size(), 0);
        }
    std::shared_ptr<std::vector<std::vector<state>>> map_t_ptr = std::make_shared<std::vector<std::vector<state>>>(map_t);
    cache.insert(std::unordered_map<std::vector<int>, std::shared_ptr<std::vector<std::vector<state>>>, map_hasher>::value_type(trail, map_t_ptr));
}

std::vector<actions>
map::actions_possible(const cv::Point &cur, const bool &sweeped)
{
    // If img.pos is marble 
    // SWEEP
    std::vector<actions> actions;
    if (cur.x - 1 >= 0)
        actions.push_back(UP);
    if (cur.x + 1 < map_size.height)
        actions.push_back(DOWN);
    if (cur.y - 1 >= 0)
        actions.push_back(LEFT);
    if (cur.y + 1 < map_size.width)
        actions.push_back(RIGHT);
    if (sweeped)
        actions.push_back(SWEEP);
    return actions;
};

void
map::print_map(std::vector<int> &trail)
{
    std::vector<std::vector<state>> *ptr_map = find_map(trail);

    for (auto &rows : (*ptr_map))
    {
        for (auto &cols : rows)
        {
            for (auto &Q_value : cols.Q_value)
            {
                std::cout << Q_value << " ";
            }
            std::cout << " , ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void
map::print_all_maps()
{
    for (size_t i = 0; i < trails.size(); i++)
    {
        std::vector<std::vector<state>> *ptr_map = find_map(trails[i]);

        for (auto &rows : (*ptr_map))
        {
            for (auto &cols : rows)
            {
                for (auto &Q_value : cols.Q_value)
                {
                    std::cout << Q_value << " ";
                }
                std::cout << " , ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

void
map::action_on_map(std::vector<int> &trail, cv::Point &pos, actions &action, double &cur_reward, double &cmf_distance)
/*
* Return the heuristic distance as a reward, the position of the action is end_pos.
*/
{
    std::vector<std::vector<state>> *ptr_map = find_map(trail);
    switch (action)
    {
    case UP:
        cur_reward = (*ptr_map)[pos.x][pos.y].reward;
        cmf_distance += cur_reward;
        pos.x -= 1;
        break;

    case DOWN:
        cur_reward = (*ptr_map)[pos.x][pos.y].reward;
        cmf_distance += cur_reward;
        pos.x += 1;
        break;

    case LEFT:
        cur_reward = (*ptr_map)[pos.x][pos.y].reward;
        cmf_distance += cur_reward;
        pos.y -= 1;
        break;

    case RIGHT:
        cur_reward = (*ptr_map)[pos.x][pos.y].reward;
        cmf_distance += cur_reward;
        pos.y += 1;
        break;

    case SWEEP:
        // The reward of a sweep is kept constant and marbles is a non constant
        trail[map_size.height * pos.x + pos.y] = map_size.height * pos.x + pos.y + 1;
        cur_reward = SWEEP_REWARD;
        break;
    }
}

std::pair<actions, double>
map::get_optimal_action(std::vector<int> &trail, cv::Point &cur)
/*
* Given Trail and position get optimal action
*/
{
    std::vector<std::vector<state>> *ptr_map = find_map(trail);
    auto max_Q = std::max_element((*ptr_map)[cur.x][cur.y].Q_value.begin(), (*ptr_map)[cur.x][cur.y].Q_value.end());
    return std::pair<actions, int>((*ptr_map)[cur.x][cur.y].actions_valid[std::distance((*ptr_map)[cur.x][cur.y].Q_value.begin(), max_Q)], *max_Q);
}

std::pair<actions, double>
map::get_random_action(std::vector<int> &trail, cv::Point &cur)
/*
* Given Trail and position get optimal action
*/
{
    static std::random_device dev;
    static std::mt19937 rng(dev());
    std::vector<std::vector<state>> *ptr_map = find_map(trail);
    int size = (*ptr_map)[cur.x][cur.y].actions_valid.size() - 1;
    std::uniform_int_distribution<std::mt19937::result_type> actionrandom(0, size);
    int random_var = actionrandom(rng);
    actions next_act = (*ptr_map)[cur.x][cur.y].actions_valid[random_var];
    return std::pair<actions, int>(next_act, (*ptr_map)[cur.x][cur.y].Q_value[random_var]);
}

void
map::update_q_value(std::vector<int> &trail, double &cur_reward, actions &action, cv::Point &pos, double &Q_value_next, double &learning_rate, double &discount_val)
{
    std::vector<std::vector<state>> *ptr_map = find_map(trail);
    for (int i = 0; i < (*ptr_map)[pos.x][pos.y].actions_valid.size(); ++i)
        if ((*ptr_map)[pos.x][pos.y].actions_valid[i] == action)
            (*ptr_map)[pos.x][pos.y].Q_value[i] += learning_rate * (cur_reward + discount_val * Q_value_next - (*ptr_map)[pos.x][pos.y].Q_value[i]);
}



/*

New section -> 

*/

#endif