#include "environment.hpp"
#include <set>
#include <string>


enum class action
{
    UP = 0,
    RIGHT = 1,
    DOWN = 2,
    LEFT = 3,
    NO_ACTION =4,
};

class Q_agent
{

public:
    Q_agent(double epsilon_max, double epsilon_min, double learning_rate, double discounted_value, environment& environment, cv::Point start_pos, int64_t max_actions_allowed=INT64_MAX);

    ~Q_agent();

    void
    learn_episodes(int episodes, bool suppres_new_episodes=true, const std::string file_name_path="");

    std::pair<int64_t, double>
    follow_optimal_route();

    void
    print_visited_environments();

    void
    print_optimal_environments();

    cv::Mat
    get_optimal_route_img(const cv::Size& resize, bool show_marbles=true, const std::string& save_path="", bool reset_route_after_show=true);

private:

    std::vector<action>& get_avaible_actions(const cv::Point& state_pos);
    action epsilon_greedy_policy(const cv::Point& state_pos);
    cv::Point get_next_state(const action& action, const cv::Point& state_pos);
    double optimal_action_policy(const cv::Point& next_state_pos);
    void update_Q_value(const cv::Point& state_pos, const cv::Point& next_state, action& action_taken, double arg_max);
    void update_Q_value(state& state, double action_reward, action& action_taken, double arg_max);
    action get_optimal_action(const cv::Point& pos);
    void print_environment_set(const std::set<std::vector<int>>& environments);

    double epsilon;
    double epsilon_max;
    double epsilon_min;
    double learning_rate;
    double discounted_value;
    environment& agent_environment;
    const cv::Point start_pos;
    int64_t max_actions_allowed;

    std::set<std::vector<int>> optimal_environments;
    std::set<std::vector<int>> visited_environments;
    std::vector<cv::Point> optimal_route;
    uint64_t actions_taken_during_episode;
    double accumulated_reward;
};