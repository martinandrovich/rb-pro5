function [mu_actions_taken, mu_accumulated_reward, alpha, epsilon, discounted] = mean_csv_files(folder_path)
if ~exist('folder_path')
    folder_path = pwd;
end

filepattern = fullfile(folder_path, '*.csv');
csv_files = dir(filepattern);
numfiles = length(csv_files);
csvdata = cell(1, numfiles);
csv_files.name;
csv_files(1).name;

for k = 1:numfiles 
    fullpathname =fullfile(folder_path,csv_files(k).name);
    csvdata{k} = csvread(fullpathname); 
end

data = [];
alpha = csvdata{1}(1,3);
epsilon = csvdata{1}(1,2);
discounted = csvdata{1}(1,4);

for i = 1:length(csvdata)
    data = [data csvdata{i}(:,1)];
end

mu_actions_taken = mean(data,2);
clear data;
data = [];

for i = 1:length(csvdata)
    data = [data csvdata{i}(:,2)];
end

mu_accumulated_reward = mean(data,2);
clear csvdata;
clear csv_files;
clear data;

end
