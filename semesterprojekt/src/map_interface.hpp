
/*
intended UI for map:


//TODO need to somehow get the optimal Q_value
from the next environment when picking up a marble.

public:
    double get_reward(const cv::Point& state_pos);
    char get_state_type(const cv::Point& state_pos);
    std::vector<std::vector<state>>& pick_up_marble(const cv::Point& marble_positon);
    void get_environment(std::vector<int>& marbles_remaining);
    void print_environment(std::vector<int>& marbles_remaining);

private:
    void generate_state_types_look_up();
    std::vector<int> generate_cache_key(std::vector<int>& remaining_marbles);
    void load_environment(cv::Mat& img);



*/





/*
intended UI for Q_learning_agent:


public:
    run_episodes(int episodes);


private:
    std::vector<actions>& get_avaible_actions(const cv::Point& state_pos);
    action& epsilon_greedy_policy(std::vector<action>& avaible_actions);
    cv::Point get_next_state(action& action, const cv::Point& state_pos);
    double optimal_action_policy(cv::Point&);
    void update_Q_value(const cv::Point& state_pos, action& action_taken);



*/
