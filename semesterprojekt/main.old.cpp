#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <unistd.h>

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core.hpp>

enum action
{
    UP = 0,
    DOWN = 1,
    RIGHT = 2,
    LEFT = 3
};


typedef struct state
{
    public:

    // Set position
    cv::Point pos;
    // Set Queue Value
    std::vector<double> Q_value;
    // Reward
    double reward;
    // Type
    char type;
} state;

class Q_learning
{

    public:

        Q_learning() : epsilon_greedy_dist(0.0f, 1.0f), generator(), random_action(0, 3)
        {
            map = std::vector<std::vector<state>>(15 , std::vector<state>( 15, { {0,0}, {0,0,0,0}, trans_reward, '-' } ) );
            std::cout << "im running " << std::endl;
            for (size_t i = 0; i < map.size(); i++)
                for (size_t j = 0; j < map[0].size(); j++)
                {
                    map[i][j].pos = cv::Point(i,j);
                    if ( i == 0 || j == 0 || i == map.size()-1 || j == map[0].size()-1 ) map[i][j].type = '#';
                    else map[i][j].type = '-';
                }
        }

        ~Q_learning()
        {
        };

        void print_map()
        {
            for (auto map_ : map)
            {
                for (auto map__ : map_)
                    std::cout << map__.type << " ";
                std::cout << std::endl;
            }
        }

        void print_Q_max_values()
        {
            for (auto map_ : map)
            {
                for (auto map__ : map_)
                    std::cout << *std::max_element(map__.Q_value.begin(), map__.Q_value.end()) << " ";
                std::cout << std::endl;
            }
        }

        void print_reward()
        {
            for (auto map_ : map)
            {
                for (auto map__ : map_)
                    std::cout << map__.reward << " ";
                std::cout << std::endl;
            }
        }

        action get_action(state & state)
        {
            // Follow the policy
            if (epsilon < epsilon_greedy_dist(generator))
            {
                //Exploit
                return (action)std::distance(state.Q_value.begin(), std::max_element(state.Q_value.begin(), state.Q_value.end()));
            }
            else
            {
                //Explore
                return (action)random_action(generator);
            }
        }
    
        state & perform_action(action & action, state & state )
        {
            if (state.pos.x < map.size()-1 && state.pos.x > 0 && state.pos.y < map[0].size()-1 && state.pos.y > 0)
            {
                /* code */
                switch (action)
                {
                    case UP:
                        if ( map[state.pos.x+1][state.pos.y].type == '#' ) break;
                        return map[state.pos.x+1][state.pos.y];

                    case DOWN:
                        if ( map[state.pos.x-1][state.pos.y].type == '#' ) break;
                        return map[state.pos.x-1][state.pos.y];

                    case RIGHT:
                        if ( map[state.pos.x][state.pos.y+1].type == '#' ) break;
                        return map[state.pos.x][state.pos.y+1];

                    case LEFT:
                        if ( map[state.pos.x][state.pos.y-1].type == '#' ) break;
                        return map[state.pos.x][state.pos.y-1];
                }
            }
            return map[state.pos.x][state.pos.y];
        }

        double get_max_Q_value(state & state )
        {
            return *std::max_element(state.Q_value.begin(), state.Q_value.end());
        }

        void episodes(int n)
        {


            map[8][8].type = '*';
            map[8][8].Q_value[0] = goal_reward; map[8][8].Q_value[1] = goal_reward; map[8][8].Q_value[2] = goal_reward; map[8][8].Q_value[3] = goal_reward;

            for (size_t i = 0; i < n; i++)
            {
                
                state * cur_state = &map[3][3];
                state * next_state;
                action first_action;
                double current_q_value = 0;
                double optimal_q_value = 0;

                while (cur_state->type != '*')
                // until it reached goal
                {
                    first_action = get_action(*cur_state);
                    next_state = &perform_action(first_action, *cur_state);
                    if ( next_state == cur_state || next_state->type == '#')
                    {
                        // means it havent moved or it hit a wall
                        cur_state->Q_value[first_action] += cur_state->reward;
                    }
                    else
                    {
                        // means it has moved
                        current_q_value = cur_state->Q_value[first_action];
                        optimal_q_value = get_max_Q_value(*next_state);
                        cur_state->Q_value[first_action] += learning_rate * (cur_state->reward + discounted_value * optimal_q_value - cur_state->Q_value[first_action]);
                    }
                    cur_state = next_state;
                }
            }
        }


    private:
        
        /* data */
        std::vector<std::vector<state>> map;

        /* specifics */
        const double epsilon                    = 0.5;
        const double learning_rate              = 0.1;
        const double trans_reward               = -0.1;
        const double goal_reward                = 10;
        const double discounted_value           = 1;
        
        /* randoms */
        std::uniform_real_distribution<double> epsilon_greedy_dist;
        std::uniform_int_distribution<int> random_action;
        std::mt19937_64 generator;
};

int main()
{

    Q_learning q_learning;
    q_learning.print_map();
    q_learning.print_reward();
    q_learning.episodes(10000);
    q_learning.print_reward();
    q_learning.print_map();
    q_learning.print_Q_max_values();
}
