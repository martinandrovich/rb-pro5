   plot_experiment("test_epsilon/", "epsilon", "actions",  "test_results_plots/vector_eps/plot_epsilon_action")
% northeast
  axis([ 85000 500000 0 30])
  saveas(gcf, "test_results_plots/vector_eps/plot_epsilon_action",'epsc')
 
 %%
 
  plot_experiment("test_epsilon/", "epsilon", "reward",  "test_results_plots/vector_eps/plot_epsilon_reward")
% southeast
  axis([ 5000 500000 0 120])
  saveas(gcf, "test_results_plots/vector_eps/plot_epsilon_reward",'epsc')

%%

plot_experiment("test_lr_0.01_epsilon/", "alpha", "reward",  "test_results_plots/vector_eps/plot_learning_rate_reward")

axis([ 5000 200000 30 120])
saveas(gcf, "test_results_plots/vector_eps/plot_learning_rate_reward",'epsc')

%%

plot_experiment("test_lr_0.01_epsilon/", "alpha", "actions",  "test_results_plots/vector_eps/plot_learning_rate_action")

axis([ 5000 500000 0 35])
saveas(gcf, "test_results_plots/vector_eps/plot_learning_rate_action",'epsc')

%%

%%

 plot_experiment("test_lr_decay_epsilon/", "alpha", "actions",  "test_results_plots/vector_eps/test_lr_decay_epsilon_action")
%  northeast
 axis([ 100000 500000 0 35])
 saveas(gcf, "test_results_plots/vector_eps/test_lr_decay_epsilon_action",'epsc')

%%

 plot_experiment("test_lr_decay_epsilon/", "alpha", "reward",  "test_results_plots/vector_eps/test_lr_decay_epsilon_reward")
% northea
 axis([ 15000 500000 40 120])
 saveas(gcf, "test_results_plots/vector_eps/test_lr_decay_epsilon_reward",'epsc')

%%

%  plot_experiment("test_lr_decay_epsilon/", "alpha", "reward",  "test_results_plots/vector_eps/test_lr_decay_epsilon_reward")
% northea
%  axis([ 15000 500000 40 120])
%  saveas(gcf, "test_results_plots/vector_eps/test_lr_decay_epsilon_reward",'epsc')

%%

plot_experiment("test_new_reference_map_static_epsilon/", "epsilon", "reward",  "test_results_plots/vector_eps/map_static_epsilon_reward")
% northea
axis([ 0 1000000 -16000 1500])
saveas(gcf, "test_results_plots/vector_eps/map_static_epsilon_reward",'epsc')

%%

plot_experiment("test_new_reference_map_static_epsilon/", "epsilon", "actions",  "test_results_plots/vector_eps/map_static_epsilon_action")
% northea
axis([ 0 1000000 0 6000000])
saveas(gcf, "test_results_plots/vector_eps/map_static_epsilon_action",'epsc')

%%

plot_experiment("test_marbles_large_unlimited_steps/", "epsilon", "actions",  "test_results_plots/vector_eps/large_unlimited_steps_action")
% northea
axis([ 0 5000000 0 60000000])
saveas(gcf, "test_results_plots/vector_eps/large_unlimited_steps_action",'epsc')
%%

plot_experiment("test_marbles_large_fixed_steps/", "epsilon", "actions",  "test_results_plots/vector_eps/large_fixed_steps_action")
% northea
axis([ 0 5000000 0 6000])
saveas(gcf, "test_results_plots/vector_eps/large_fixed_steps_action",'epsc')

