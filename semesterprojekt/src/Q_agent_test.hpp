#include "Q_agent.hpp"



bool test_episodes(int iterations , const std::string& path , environment& env, int episodes, double epsilon_max, double epsilon_min, double alpha, double discount, int64_t max_actions=INT64_MAX, cv::Point startpos={60,40})
{
    Q_agent agent(epsilon_max, epsilon_min, alpha, discount, env, startpos, max_actions);
    cv::Mat img;
    for(int i = 0; i < iterations; i++)
    {
        env.reset_all_environments();
        agent.learn_episodes(episodes, false, path + std::to_string(i) + "_" );
        img = agent.get_optimal_route_img(cv::Size(5, 5), true, path + std::to_string(i) + "_");
    }
    //cv::imshow("Trained_agent optimal path", img);
    //cv::waitKey();
    return true;
}

long long signed operator "" _k(long double value) {return value*1000;}

void test_for_report()
{
    cv::Mat src, src_gray, binary;
    src = cv::imread("floor_plan.png");
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::threshold(src_gray, binary, 250, 255, CV_THRESH_BINARY);

    environment floor_plan(-0.1, -1, 20, binary, 0.);
    floor_plan.add_marble({45, 25});
    floor_plan.add_marble({15, 26});
    floor_plan.add_marble({109, 45});
    floor_plan.add_marble({103, 29});

    int64_t episodes = 500._k;

    //test_episodes(10, "report_experiments/test_lr/test1/_", floor_plan, episodes, 0.1, 0.01, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr/test2/_", floor_plan, episodes, 0.1, 0.01, 0.20, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr/test3/_", floor_plan, episodes, 0.1, 0.01, 0.30, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr/test4/_", floor_plan, episodes, 0.1, 0.01, 0.40, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr/test5/_", floor_plan, episodes, 0.1, 0.01, 0.50, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr/test6/_", floor_plan, episodes, 0.1, 0.01, 0.60, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr/test7/_", floor_plan, episodes, 0.1, 0.01, 0.70, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr/test8/_", floor_plan, episodes, 0.1, 0.01, 0.80, 0.9, 242);


    //test_episodes(10, "report_experiments/test_lr_0.01_epsilon/test1/_", floor_plan, episodes, 0.01, -0.01, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_0.01_epsilon/test2/_", floor_plan, episodes, 0.01, -0.01, 0.20, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_0.01_epsilon/test3/_", floor_plan, episodes, 0.01, -0.01, 0.30, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_0.01_epsilon/test4/_", floor_plan, episodes, 0.01, -0.01, 0.40, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_0.01_epsilon/test5/_", floor_plan, episodes, 0.01, -0.01, 0.50, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_0.01_epsilon/test6/_", floor_plan, episodes, 0.01, -0.01, 0.60, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_0.01_epsilon/test7/_", floor_plan, episodes, 0.01, -0.01, 0.70, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_0.01_epsilon/test8/_", floor_plan, episodes, 0.01, -0.01, 0.80, 0.9, 242);


    //test_episodes(10, "report_experiments/test_lr_decay_epsilon/test1/_", floor_plan, episodes, 0.25, 0.001, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_decay_epsilon/test2/_", floor_plan, episodes, 0.25, 0.001, 0.20, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_decay_epsilon/test3/_", floor_plan, episodes, 0.25, 0.001, 0.30, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_decay_epsilon/test4/_", floor_plan, episodes, 0.25, 0.001, 0.40, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_decay_epsilon/test5/_", floor_plan, episodes, 0.25, 0.001, 0.50, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_decay_epsilon/test6/_", floor_plan, episodes, 0.25, 0.001, 0.60, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_decay_epsilon/test7/_", floor_plan, episodes, 0.25, 0.001, 0.70, 0.9, 242);
    //test_episodes(10, "report_experiments/test_lr_decay_epsilon/test8/_", floor_plan, episodes, 0.25, 0.001, 0.80, 0.9, 242);
//
    ////in order to test this we need to find the best value for the paramter of learning rate.
    //test_episodes(10, "report_experiments/test_epsilon/test1/_", floor_plan, episodes, 0.01, 0.01, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_epsilon/test2/_", floor_plan, episodes, 0.1, 0.01, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_epsilon/test3/_", floor_plan, episodes, 0.2, 0.01, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_epsilon/test4/_", floor_plan, episodes, 0.3, 0.01, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_epsilon/test5/_", floor_plan, episodes, 0.4, 0.01, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_epsilon/test6/_", floor_plan, episodes, 0.5, 0.01, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_epsilon/test7/_", floor_plan, episodes, 0.6, 0.01, 0.10, 0.9, 242);

    //test_episodes(10, "report_experiments/test_discount/test1/_", floor_plan, episodes, 0.1, -0.01, 0.10, 0.95, 242);
    //test_episodes(10, "report_experiments/test_discount/test2/_", floor_plan, episodes, 0.1, -0.01, 0.10, 0.9, 242);
    //test_episodes(10, "report_experiments/test_discount/test3/_", floor_plan, episodes, 0.1, -0.01, 0.10, 0.85, 242);
    //test_episodes(10, "report_experiments/test_discount/test4/_", floor_plan, episodes, 0.1, -0.01, 0.10, 0.8, 242);
    //test_episodes(10, "report_experiments/test_discount/test5/_", floor_plan, episodes, 0.1, -0.01, 0.10, 0.75, 242);
    //test_episodes(10, "report_experiments/test_discount/test6/_", floor_plan, episodes, 0.1, -0.01, 0.10, 0.7, 242);
    //test_episodes(10, "report_experiments/test_discount/test7/_", floor_plan, episodes, 0.1, -0.01, 0.10, 0.3, 242);
    episodes = 10000._k;
    test_episodes(1, "report_experiments/test_new_reference_map/test1/", floor_plan, episodes, 0.1, -0.1, 0.15, 0.9);
    //cv::Mat resized;
    //cv::resize(floor_plan.get_environment_img(), resized, cv::Size(), 5, 5, cv::INTER_NEAREST);
    //cv::imwrite("report_experiments/test_reference_map/test1/reference_map_marble_locations.png", resized);

    //floor_plan.add_marble({9, 8});
    //floor_plan.add_marble({24, 9});
    //floor_plan.add_marble({16, 24});
    //floor_plan.add_marble({41, 11});
    //floor_plan.add_marble({70, 10});
    //floor_plan.add_marble({10, 65});
    //floor_plan.add_marble({32, 61});
    //floor_plan.add_marble({54, 71});
    //floor_plan.add_marble({77, 70});
    //floor_plan.add_marble({111, 10});
    //floor_plan.add_marble({37, 40});
    //floor_plan.add_marble({79, 38});
    //floor_plan.add_marble({91, 18});
    //floor_plan.add_marble({106, 70});
    //floor_plan.add_marble({88, 50});
    //floor_plan.add_marble({57, 25});
    //floor_plan.add_marble({77, 27});
    //cv::Mat resized;
    //cv::resize(floor_plan.get_environment_img(), resized, cv::Size(), 5, 5, cv::INTER_NEAREST);
    //cv::imwrite("report_experiments/20_marbles_large_test.png", resized);
    //test_episodes(1, "report_experiments/_20_marbles_large_test_", floor_plan, episodes, 0.1, -0.01, 0.15, 0.9);


}




bool test_10x10_Q_agent()
{
    cv::Mat src, src_gray, binary;
    src = cv::imread("pic_10x10.png");
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::threshold(src_gray, binary, 250, 255, CV_THRESH_BINARY);
    environment small_world(-0.1, -1, 0, binary, 0);
    small_world.add_marble({1, 8});
    small_world.add_marble({2, 8});
    small_world.add_marble({1, 9});

    small_world.add_marble({9, 4});
    int64_t episodes = 500000;

    Q_agent agent(0.8, 0.1, 0.1, 0.9, small_world, {8,8}, 12);
    cv::Mat img;
    int iterations = 1;
    for(int i = 0; i < iterations; i++)
    {
        small_world.reset_all_environments();
        small_world.print_environment_type_map();
        agent.learn_episodes(episodes, false, "small_world_test/_" + std::to_string(i) + "_" );
        agent.print_optimal_environments();
        agent.print_visited_environments();
        img = agent.get_optimal_route_img(cv::Size(500, 500), true, "small_world_test/_"  + std::to_string(i) + "_");
    }

    //test_episodes(10, "small_world_test/_", small_world, episodes, 0.8, 0.05, 0.15, 0.9, 12, {8,8});
    return true;
}

bool test_Q_agent()
{
    cv::Mat src, src_gray, binary;
    src = cv::imread("floor_plan.png");
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::threshold(src_gray, binary, 250, 255, CV_THRESH_BINARY);

    environment floor_plan(-0.1, -1, 2, binary, 0.);

    //floor_plan.add_marble({88, 37});
    floor_plan.add_marble({95, 33});
    floor_plan.add_marble({10, 39});
    floor_plan.add_marble({10, 38});
    floor_plan.add_marble({11, 45});
    floor_plan.add_marble({8, 50});
    floor_plan.add_marble({4, 37});

    std::cout << " running test.." << std::endl;
    int64_t episodes = 75000;
    //test_episodes(10 ,"tuning/_" ,floor_plan, episodes, 0.1 , 0.1, 0.9, 1000);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.01, 0.1, 0.99, 170);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.01, 0.1, 0.7, 170);

    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.4, 0.90, 170);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.3, 0.90, 170);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.2, 0.90, 170);

    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.2, 0.1, 0.90, 170);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.3, 0.1, 0.90, 170);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.3, 0.1, 0.90, 170);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.4, 0.1, 0.90, 170);
//
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.1, 0.90, 140);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.1, 0.90, 130);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.1, 0.90, 120);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.1, 0.90, 110);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.1, 0.90, 100);
//
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.1, 0.90, 1000);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.1, 0.2, 0.90, 1000);
    //test_episodes(10 ,"experiments/_" ,floor_plan, episodes, 0.2, 0.2, 0.90, 1000);
    //test_episodes(1, "tuning/_", floor_plan, episodes, 0.5, 0.01, 0.15, 0.9, 120);
    test_for_report();
    //test_10x10_Q_agent();
    std::cout << " running test.. done." << std::endl;

    return true;
}




//Ideas:
//Try to give a reward when it's done based on how many marbles that have been collected.
//Optimize the optimal random dist.
//Give a fixed amount of steps within each new environment instead of a total amount of steps taken.
//