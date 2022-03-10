
clear
clear global
close all
clc

addpath(genpath(pwd));

rng(0);

num_cases = 1;

max_x = 100;
max_y = 100;
max_theta = 2*pi;

ran_base_point = [31, 98];

ran_start_pose = [134, 95, 0];

ran_goal_pose = [108, 154];

global map_filename;
map_filename = 'maps/mymap3_matlab.png';
global max_cable_length;
max_cable_length = 120;

global Mmap;
global critical_points;
[Mmap, critical_points] = generatePolyMap(map_filename);

% Generate robot footprint (grid size)
% The robot is 50cm * 50cm while the grid resolution of map is 5cm
footprint = [5, 0; -5, -5; -5, 5; -5, -5; 5, -5; 5, 5; -5, 5];

% Calculate the admssible tether state
global phi_1;
phi_1 = wrapTo2Pi(atan2(footprint(end, 2)-footprint(1, 2), footprint(end, 1)-footprint(1, 1)));
phi_1 = phi_1 + 0.05;

global phi_2;
phi_2 = wrapTo2Pi(atan2(footprint(2, 2)-footprint(1, 2), footprint(2, 1)-footprint(1, 1)));
phi_2 = phi_2 - 0.05;

% Refine the robot footprint for collision checking
n = 4;
temp = [];
footprint = [footprint; footprint(1, :)];
for i = 1:size(footprint, 1)-1
    tempx = [linspace(footprint(i, 1), footprint(i+1, 1), n)]';
    tempy = [linspace(footprint(i, 2), footprint(i+1, 2), n)]';
    temp = [temp; [tempx(1:end-1, :), tempy(1:end-1, :)]];    
end
footprint = temp;

solution_paths = cell(num_cases, 1);
for i = 1:num_cases
    solution_paths{i, 1} = planner_wrapper(footprint, round(ran_base_point(i, :)), round(ran_start_pose(i, :)), round(ran_goal_pose(i, :)));
end

forPaperVisualization
