#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <unistd.h>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <fstream>

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core.hpp>

#define STOP 10

enum actions
{
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    SWEEP = 4,
};

typedef struct state
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

} state;

typedef struct vectorhasher
{
    int operator()(const std::vector<int> &V) const
    {
        int hash = 0;
        for (int i = 0; i < V.size(); i++)
            hash += V[i]; // Can be anything
        return hash;
    }
} vectorhasher;

class map
{
public:
    map(double reward_, cv::Size map_size_) : map_size(map_size_), marbles_in_each_area(0, 1), reward(reward_), cache(), img(map_size, CV_8UC1)
    {
        std::cout << "Init map... \n";

        // Create Vector Maps
        create_maps();

        //Update end map;
        update_end_map();

        // Create a map from openCV
        img = cv::Mat::zeros(map_size, CV_8UC1);

        std::cout << "Map is created... " << std::endl;
    }

    ~map(){};

    inline bool
    find_map_t_ptr(const std::vector<int> &trail)
    /*
    * 
    */
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

    void
    update_end_map()
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

    inline void
    create_maps()
    /*
    * 
    */
    {
        int trail_start_idx = 0, trail_end_idx = 1, iterate_val = 0;
        auto trail = std::vector<int>(map_size.height * map_size.width, 0);
        create_one_map(trail, iterate_val);
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
                            create_one_map(trial_new, iterate_val);
                        }
                        trial_new[j] -= j + 1;
                    }
                }
            }

            trail_start_idx = trail_end_idx;
            trail_end_idx = trails.size();
        }
        trails.clear();
    }

    inline void
    create_one_map(std::vector<int> &trail, int m)
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
                map_t[i][j].sweeped = (trail[3 * i + j] != 0) ? 0 : 1;
                map_t[i][j].actions_valid = actions_possible(map_t[i][j].pos, map_t[i][j].sweeped);
                map_t[i][j].Q_value = std::vector<double>(map_t[i][j].actions_valid.size(), 0);
            }
        std::shared_ptr<std::vector<std::vector<state>>> map_t_ptr = std::make_shared<std::vector<std::vector<state>>>(map_t);
        cache.insert(std::unordered_map<std::vector<int>, std::shared_ptr<std::vector<std::vector<state>>>, vectorhasher>::value_type(trail, map_t_ptr));
    }

    std::vector<actions>
    actions_possible(cv::Point &cur, bool &sweeped)
    {
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

    std::vector<std::vector<state>> *
    find_map(std::vector<int> &trail)
    /*
    * Look up the map, by using current trail Remember the trail consists of more information
    */
    {
        auto got = cache.find(trail);
        while (got != cache.end())
        {
            if (got->first == trail)
            {

                return (got->second.get());
            }
            got++;
        }
        std::cout << "ERROR" << std::endl;
        return got->second.get();
    }

    void
    print_map(std::vector<int> &trail)
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
    action_on_map(std::vector<int> &trail, cv::Point &pos, actions &action, double &cur_reward, double &cmf_distance)
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
            trail[3 * pos.x + pos.y] = 3 * pos.x + pos.y + 1;
            cur_reward = 1;
            break;
        }
    }

    std::pair<actions, double>
    get_optimal_action(std::vector<int> &trail, cv::Point &cur)
    /*
    * Given Trail and position get optimal action
    */
    {
        std::vector<std::vector<state>> *ptr_map = find_map(trail);
        auto max_Q = std::max_element((*ptr_map)[cur.x][cur.y].Q_value.begin(), (*ptr_map)[cur.x][cur.y].Q_value.end());
        return std::pair<actions, int>((*ptr_map)[cur.x][cur.y].actions_valid[std::distance((*ptr_map)[cur.x][cur.y].Q_value.begin(), max_Q)], *max_Q);
    }

    std::pair<actions, double>
    get_random_action(std::vector<int> &trail, cv::Point &cur)
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
    update_q_value(std::vector<int> &trail, double &cur_reward, actions &action, cv::Point &pos, double &Q_value_next, double &learning_rate, double &discount_val)
    {
        std::vector<std::vector<state>> *ptr_map = find_map(trail);
        for (int i = 0; i < (*ptr_map)[pos.x][pos.y].actions_valid.size(); ++i)
            if ((*ptr_map)[pos.x][pos.y].actions_valid[i] == action)
                (*ptr_map)[pos.x][pos.y].Q_value[i] += learning_rate * (cur_reward + discount_val * Q_value_next - (*ptr_map)[pos.x][pos.y].Q_value[i]);
    }

private:
    std::unordered_map<std::vector<int>, std::shared_ptr<std::vector<std::vector<state>>>, vectorhasher> cache;
    std::uniform_real_distribution<double> marbles_in_each_area;

    cv::Size map_size;
    double reward;
    cv::Mat img;
    std::vector<std::vector<double>> marbles_in_pos;
    cv::Point recharge_station;
    std::vector<std::vector<int>> trails;
};

class Q_learning
{

public:
    Q_learning(double reward_, cv::Size map_size_) : map_size(map_size_), map_t(reward_, map_size_)
    {
        // Enter new map

        do_episodes_learning(2000);
    }

    ~Q_learning(){

    };

    void
    do_episodes_learning(int episodes)
    {
        static std::random_device dev;
        static std::mt19937 rng(dev());
        std::uniform_real_distribution<> epsilon_greedy_dist(0.0f, 1.0f);

        std::ofstream fs("matlab.csv");

        if (!fs.is_open())
        {
            std::cerr << "Error could not open: matlab.csv" << std::endl;
            return;
        }

        for (size_t i = 0; i < episodes; i++)
        {
            /* Start position is always (1,1) */
            cv::Point current(0, 0);

            cv::Point nextpos;

            /* Heurstic distance travelled */
            distance_travelled = 0;

            /* R_{t+1} */
            double cur_reward;

            /* Start trail with no sweeped */
            trail = std::vector<int>(map_size.height * map_size.width, 0);
            newtrail = std::vector<int>(map_size.height * map_size.width, 0);

            auto next_action = map_t.get_optimal_action(trail, current);

            bool run = true;

            while (run)
            {
                if (epsilon_greedy_dist(rng) > epsilon)
                    /* Based on state action pairs */
                    next_action = map_t.get_optimal_action(trail, current);
                else
                    next_action = map_t.get_random_action(trail, current);

                nextpos = current;
                newtrail = trail;

                /* Returns reward and new position */
                map_t.action_on_map(newtrail, nextpos, next_action.first, cur_reward, distance_travelled);

                /* Optimal Action once again */
                auto arg_max = map_t.get_optimal_action(newtrail, nextpos);

                map_t.update_q_value(trail, cur_reward, next_action.first, current, arg_max.second, learning_rate, discounted_value);

                current = nextpos;
                trail = newtrail;

                if (next_action.second == STOP || arg_max.second == STOP)
                {
                    run = false;
                }
            }
            fs << distance_travelled << '\n';
        }

        fs.close();
    }

    void
    follow_optimal_route()
    {
        /* Start position is always (1,1) */
        cv::Point current(1, 1);

        cv::Point nextpos;

        /* Heurstic distance travelled */
        distance_travelled = 0;

        /* R_{t+1} */
        double cur_reward;

        /* Start trail with no sweeped */
        trail = std::vector<int>(map_size.height * map_size.width, 0);
        newtrail = std::vector<int>(map_size.height * map_size.width, 0);

        auto next_action = map_t.get_optimal_action(trail, current);

        bool run = true;

        while (run)
        {
            next_action = map_t.get_optimal_action(trail, current);

            /* Returns reward and new position */
            map_t.action_on_map(trail, current, next_action.first, cur_reward, distance_travelled);

            if (next_action.second == 1)
            {
                run = false;
            }
        }

        std::cout << distance_travelled << std::endl;
    }

private:
    /* map */
    map map_t;

    /* specifics */

    double epsilon = 0.01;
    double learning_rate = 0.1;
    double reward = -0.1;
    double discounted_value = 1;
    double distance_travelled;

    std::vector<int> trail;
    std::vector<int> newtrail;

    cv::Size map_size;
};

int main()
{
    Q_learning(-0.1, cv::Size(3, 3));
}
