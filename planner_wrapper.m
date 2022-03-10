function resulting_path = planner_wrapper(footprint, base_point, start_poseXYZ, goal_poseXY)

% We store the goal location to a global variable, so that we need not pass
% it through function parameters
global goal_loc;
goal_loc = goal_poseXY;

global Mmap phi_1 phi_2;

if Mmap(base_point(1), base_point(2)) ~= 0
    fprintf('The base_point is in the obstacle.\n');
    resulting_path = [];
    return ;
end

% The first footprint point is the anchor point
global polar_footprint;
polar_footprint = zeros(size(footprint, 1), 2);
for i = 1:size(footprint, 1)
    polar_footprint(i, 1) = norm(footprint(i, :));
    polar_footprint(i, 2) = atan2(footprint(i, 2), footprint(i, 1));
end

% Just pre-check
if ~isCollisionFree(Mmap, polar_footprint, start_poseXYZ(1), start_poseXYZ(2), start_poseXYZ(3))
    fprintf('Starting pose is in collision.\n');
    resulting_path = [];
    return ;
end
if ~isCollisionFree(Mmap, polar_footprint, goal_poseXY(1), goal_poseXY(2), 0)
    fprintf('Goal pose is in collision.\n');
    resulting_path = [];
    return ;
end
    
% We find the initial tethered robot configuration
% We use the naive dijkstra to generate a usable initial configuration
initial_path = dijkstra_4dir(Mmap, base_point, start_poseXYZ(1:2));
initial_cable = [base_point];
for i = 1:size(initial_path, 1)-1
    initial_cable = getCableState(initial_cable, initial_path(i, :), initial_path(i+1, :));
end

% After finding all obstacle vertices, we set the base_point (anchoring the
% cable) to be an obstacle. So that the robot won't visit this point, and
% the cable state is well-defined. 
temp = base_point + [-1, -1; -1, 0; -1, 1; 0, -1; 0, 0; 0, 1; 1, -1; 1, 0; 1, 1];
for i = 1:size(temp, 1)
    Mmap(temp(i, 1), temp(i, 2)) = 254;
end

start_phi = atan2(initial_cable(end, 2)-start_poseXYZ(2), initial_cable(end, 1)-start_poseXYZ(1));

%% We create the initial configuration
start_pose = struct('index', 1, 'x', start_poseXYZ(1), 'y', start_poseXYZ(2), 'theta', start_phi-(phi_1+phi_2)/2, 's', [], 'phi', start_phi, ...
    'cost', 0, 'h', norm(start_poseXYZ(1:2)-goal_poseXY), 'hindex', [], 'steer', 0, 'motion_direction', 1, 'open', true, 'fatherindex', -1, ...
    'obs_vertices', [initial_cable], 'mid_poses', []);
start_pose.s = polarRotateAndMoveToXy(polar_footprint(1, :), start_pose.x, start_pose.y, start_pose.theta);

resulting_path = planner_main(start_pose);

end