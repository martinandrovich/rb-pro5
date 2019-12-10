#include "Q_agent.hpp"
#include <fstream>
#include <sstream>


Q_agent::Q_agent(double epsilon_max, double epsilon_min, double learning_rate, double discounted_value, environment& environment, cv::Point start_pos, int64_t max_actions_allowed)
: epsilon{epsilon_max}
, epsilon_max{epsilon_max}
, epsilon_min{epsilon_min}
, learning_rate{learning_rate}
, discounted_value{discounted_value}
, agent_environment{environment}
, start_pos{start_pos}
, max_actions_allowed{max_actions_allowed}
, optimal_environments()
, visited_environments()
, optimal_route()
, actions_taken_during_episode{0}
, accumulated_reward{0}
{
    std::cout << "Q_agent initialized" << std::endl;
}

Q_agent::~Q_agent()
{}

void
Q_agent::learn_episodes(int episodes, bool suppres_new_episodes, const std::string file_name_path)
{
    std::ofstream learning_data;
    if(file_name_path != "")
    {
        std::ostringstream streamobj;
        streamobj << std::setprecision(2) << "Eps_max_"  <<  epsilon_max << "_Eps_min_" << epsilon_min <<  "_ln_"  << learning_rate << "_dis_" << discounted_value << "_max_actions_" << max_actions_allowed << ".csv";
        std::string test_name = file_name_path + streamobj.str();
        learning_data.open(test_name, std::ios::out);
        learning_data << episodes << "," << epsilon_max << "," <<
        learning_rate << "," << discounted_value << std::endl;
    }

    for(int i = 0; i < episodes; i++)
    {
        auto& new_episode_environment = agent_environment.init_environment();
        cv::Point current_state = start_pos;
        bool terminate = false;
        if(!suppres_new_episodes)
        {
            if(i%10000 == 0)
            {
                std::cout << "Episodes done: " << i  << "   actions_taken_during episode: "
                << actions_taken_during_episode << "  unique environments discovered so far: "
                << agent_environment.get_total_environments_discovered()<< "   Epsilon is: " << epsilon << "   Accumulated reward: " << accumulated_reward << std::endl;
            }
        }

        actions_taken_during_episode = 0;
        accumulated_reward = 0;
        //Disabled if epsilon_min is < 0 -> only use the epsilon_max setting.
        if(epsilon > epsilon_min && epsilon_min >= 0)
        {
            if(epsilon <= 0) epsilon = 0;
            else epsilon -=  epsilon_max/(episodes);
        }
        //std::cout << "Epsilon is: " <<  epsilon << " at episode: " <<  i << ":" << episodes << std::endl;

        while(!terminate && actions_taken_during_episode < max_actions_allowed)
        {
            actions_taken_during_episode++;
            action greedy_action = epsilon_greedy_policy(current_state);
            cv::Point next_state = get_next_state(greedy_action, current_state);

            switch (agent_environment.get_state_type(next_state))
            {
                case WALL_TYPE:
                {
                    update_Q_value(current_state, next_state, greedy_action, -10);
                    break;
                }

                case MARBLE_TYPE:
                {
                    /* Need to get acces to the current state, because when the marble is picked up, the environment will change.
                    This is to allow Q_values to traverse in between multiple environments. */
                    const auto& env_key = agent_environment.get_current_remaining_marbles();
                    visited_environments.insert(env_key);

                    auto& Q_values_current_state = agent_environment.get_state(current_state);
                    double marble_reward = agent_environment.get_reward(next_state);

                    auto& current_map = agent_environment.get_environment(env_key);

                    environment_map_t* new_environment = &agent_environment.pick_up_marble(next_state);

                    //When a marble has been picked up, the environment changes accordingly. ie. further calls to optimal_action_policy and update_Q_value will be based on a "new" environment.
                    double arg_max_next_environment = optimal_action_policy(next_state);
                    update_Q_value(Q_values_current_state, marble_reward, greedy_action, arg_max_next_environment);
                    current_state = next_state;

                    if(agent_environment.get_current_remaining_marbles().size() == 0)
                    {
                        terminate = true;
                    }
                    break;
                }

                case WALKABLE_TYPE:
                {
                    double arg_max = optimal_action_policy(next_state);
                    update_Q_value(current_state, next_state, greedy_action, arg_max);
                    current_state = next_state;
                    break;
                }

                default:
                {
                      std::cout << "Error occured DEFAULT SHOULD NEVER HAPPEN" << std::endl;
                    break;
                }
            }
        }
        if(i%500 == 0)
        {
            learning_data << actions_taken_during_episode << "," << accumulated_reward << std::endl;
        }
    }
    std::cout << episodes <<" episodes done..." << std::endl;
    epsilon = epsilon_max;
    auto optimal_action_and_reward = follow_optimal_route();
    learning_data << optimal_action_and_reward.first << "," << optimal_action_and_reward.second;
    learning_data.close();
}

std::pair<int64_t, double>
Q_agent::follow_optimal_route()
{
    std::cout << "Finding optimal route" << std::endl;
    agent_environment.init_environment();
    bool terminate = false;
    cv::Point curent_state = start_pos;
    int64_t actions_taken_during_run = 0;
    double accumulated_reward = 0;
    while(!terminate &&  actions_taken_during_run < max_actions_allowed)
    {
        actions_taken_during_run++;
        optimal_route.push_back(curent_state);
        action optimal_action = get_optimal_action(curent_state);
        cv::Point next_state = get_next_state(optimal_action, curent_state);
        auto state_type = agent_environment.get_state_type(next_state);
        accumulated_reward += agent_environment.get_reward(next_state);

        if(state_type == MARBLE_TYPE)
        {
            optimal_environments.insert(agent_environment.get_current_remaining_marbles());
            agent_environment.pick_up_marble(next_state);
            if(agent_environment.get_current_remaining_marbles().size() == 0) terminate = true;
        }
        if(state_type == WALL_TYPE)
        {
            continue;
        }
        curent_state = next_state;
    }
    std::cout << "Done finding route. actions took during route: " << actions_taken_during_run << std::endl;
    return {actions_taken_during_run, accumulated_reward};
}

void
Q_agent::print_visited_environments()
{
    std::cout << "Printing visited environments" << std::endl;
    print_environment_set(visited_environments);
}

void
Q_agent::print_optimal_environments()
{
    std::cout << "Printing optimal environments" << std::endl;
    print_environment_set(optimal_environments);
}

action
Q_agent::epsilon_greedy_policy(const cv::Point& state_pos)
{
    static std::random_device rand_dev;
    static std::mt19937 rng(rand_dev());
    //static std::uniform_real_distribution<double> epsilon_greedy_dist(0.0, 1.0);
    static std::uniform_int_distribution<std::mt19937::result_type> rand_action(0, 3);
    static std::uniform_int_distribution<std::mt19937::result_type> epsilon_greedy_dist(0, 100000);

    //TODO make a tolerance of what Q_values are seen as the same. -> discretize.
    double greedy =(double)epsilon_greedy_dist(rng)/100000.0;

    if(greedy >= epsilon)
    {
       return get_optimal_action(state_pos);
    }
    else
    {
        return (action)rand_action(rng);
    }
}

cv::Point
Q_agent::get_next_state(const action& action, const cv::Point& state_pos)
{
    switch (action)
    {
    case action::UP:
        return {state_pos.x, state_pos.y - 1};

    case action::RIGHT:
        return {state_pos.x + 1, state_pos.y};

    case action::DOWN:
        return {state_pos.x, state_pos.y + 1};

    case action::LEFT:
        return {state_pos.x - 1, state_pos.y};

    default:
       return state_pos;
    }
}

double
Q_agent::optimal_action_policy(const cv::Point& next_state_pos)
{
    auto& state = agent_environment.get_state(next_state_pos);
    return *std::max_element(state.Q_value.begin(), state.Q_value.end());
}

void
Q_agent::update_Q_value(const cv::Point& state_pos, const cv::Point& next_state, action& action_taken, double arg_max)
{
    auto& state = agent_environment.get_state(state_pos);
    auto reward = agent_environment.get_reward(next_state);
    accumulated_reward += reward;
    state.Q_value[(int)action_taken] += learning_rate * (reward + discounted_value * arg_max - state.Q_value[(int)action_taken]);
}

void
Q_agent::update_Q_value(state& state, double action_reward, action& action_taken, double arg_max)
{
    accumulated_reward += action_reward;
    state.Q_value[(int)action_taken] += learning_rate * (action_reward + discounted_value * arg_max - state.Q_value[(int)action_taken]);
}

action
Q_agent::get_optimal_action(const cv::Point& state_pos)
{
    const auto state = agent_environment.get_state(state_pos);
    auto cpy = state;
    std::sort(cpy.Q_value.begin(),cpy.Q_value.end());
    //for(auto)
    return (action)std::distance(state.Q_value.begin(), std::max_element(state.Q_value.begin(),state.Q_value.end()));
}

void
Q_agent::print_environment_set(const std::set<std::vector<int>>& environments)
{
    for(auto& env_key : environments)
    {
        std::cout << "Remaining marbles: ";
        for(const auto& position: env_key)
        {
            std::cout << position << " , ";
        }
        std::cout << std::endl;
        agent_environment.print_environment_Q_optimal( agent_environment.get_environment(env_key));
    }
}

cv::Mat
Q_agent::get_optimal_route_img(const cv::Size& resize, bool show_marbles, const std::string& save_path, bool reset_route_after_show)
{
    auto env_img = agent_environment.get_environment_img(show_marbles);
    cv::Mat resized_img;
    for(const auto& point: optimal_route)
    {
        env_img.at<cv::Vec3b>(point) = cv::Vec3b(0, 255, 0);
    }

    cv::resize(env_img, resized_img, cv::Size(), resize.width, resize.height, cv::INTER_NEAREST);
    if(save_path != "")
    {
        std::ostringstream streamobj;
        streamobj << std::setprecision(2) << "Eps_max_"  <<  epsilon_max << "_Eps_min_" << epsilon_min <<  "_ln_"  << learning_rate << "_dis_" << discounted_value << "_max_actions_" << max_actions_allowed  << "_img_" << ".png";
        std::string img_name = save_path + streamobj.str();
        try
        {
            std::cout << "Saving image.. as: " << img_name << std::endl;
            imwrite(img_name, resized_img);
        }
        catch (std::runtime_error& ex)
        {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
        }
    }
    if(reset_route_after_show) optimal_route.clear();
    return resized_img;
}