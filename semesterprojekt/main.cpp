#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <unistd.h>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <fstream>
#include "src/map.hpp"
#include "src/map_test.hpp"
#include "src/Q_agent_test.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>





#if 0
class Q_learning
{

public:
    Q_learning(double reward_, cv::Size map_size_)
    : map_size(map_size_)
    , map_t(reward_, map_size_)
    {
        // Enter new map

        do_episodes_learning(5000);

        //map_t.print_all_maps();
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

                //map_t.print_map(trail);

                if (next_action.second == STOP || arg_max.second == STOP)
                {
                    run = false;
                }
            }
            fs << distance_travelled << '\n';

            //std::cout << i << std::endl;
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

    double epsilon = 0.05;
    double learning_rate = 0.1;
    double reward = -0.1;
    double discounted_value = 1;
    double distance_travelled;

    std::vector<int> trail;
    std::vector<int> newtrail;

    cv::Size map_size;
};

#endif

#if !TEST

int main()
{
    environment DUT();
}

#endif

#if TEST

    int main()
    {

        std::cout << "Test running" << std::endl;
        //run_test();
        test_Q_agent();

        return 0;
    }

#endif