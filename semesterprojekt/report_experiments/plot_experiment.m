function [] = plot_experiment(folder_path, type, y_label, name)
minimum_y_data = 100000000000;
maximum_y_data= -100000000000;
set(0, 'DefaultFigureRenderer', 'painters');

legend_ = [];
choose_y_data = [];

filepattern = fullfile(folder_path, "*test*");
subfolders = dir(filepattern);
num_folders = length(subfolders);
subfolders.name;

for i = 1:num_folders
    [actions, reward,  alpha, epsilon, discounted] = mean_csv_files(folder_path + subfolders(i).name + '/');
    
    switch type
          case 'alpha'
            legend_ = [legend_ '$\alpha = ' + string(alpha) + '$'];
          case 'epsilon'
            legend_ = [legend_ '$\epsilon = ' + string(epsilon) + '$'];
          case'gamma'
            legend_ = [legend_ '$\gamma = ' + string(discounted) + '$'];
          otherwise
    end
    
    switch y_label
        case 'actions'
            choose_y_data = actions(2:end, 1);
%             minimum_y_data = 202;
            
        
        case 'reward'
            choose_y_data = reward(2:end -1 , 1);
%             maximum_y_data = 60.2;
            
    end
                  
    % determine minimum reward first.
    if ( min(choose_y_data) < minimum_y_data )
        minimum_y_data = min(choose_y_data);
    end
    
    % determine highest reward
    if ( max(choose_y_data) >  maximum_y_data )
        maximum_y_data = max(choose_y_data);
    end
end
disp(maximum_y_data)
disp(minimum_y_data)

if(y_label == "reward")
    maximum_y_data = maximum_y_data + abs(minimum_y_data);
end
disp(maximum_y_data)
disp(minimum_y_data)



clf;

for i = 1:num_folders
    [actions, reward,  alpha, epsilon, discounted] = mean_csv_files(folder_path + subfolders(i).name + '/');
   
   switch y_label
        case 'actions'
              choose_y_data = actions(2:end-1, 1);
              choose_y_data =( ((choose_y_data(2:end - 1, 1))./minimum_y_data) * 100) - 100;
%             new_max = max(abs(choose_y_data(2:end - 1, 1) - maximum_y_data))
%             choose_y_data =( abs(choose_y_data(2:end - 1, 1) - maximum_y_data ) )./new_max * 100;
                    
        case 'reward'
              choose_y_data = reward(2:end -1 , 1);
              choose_y_data = (choose_y_data(2:end - 1, 1) + abs(minimum_y_data))./maximum_y_data * 100;
   end
    semilogx( 1:500:(length(choose_y_data))*500, choose_y_data);
    %%loglog( 1:500:(length(choose_y_data))*500, choose_y_data);
    hold on;
    
end

switch y_label
    case 'actions'
         % y_label = y_label + ' error \% relative to optimal';
        
    case 'reward'
         % y_label = y_label + ' \% relative to optimal';
end

xlabel('episode ', 'interpreter','latex')
ylabel(y_label, 'interpreter','latex')
% legend(legend_, 'interpreter','latex','location', 'southeast')
legend(legend_, 'interpreter','latex','location', 'northeast')
saveas(gcf, name,'epsc')

hold off;
