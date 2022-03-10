function sons_node = get_neighbors(cur_node)

MIN_STEER = -1;
MAX_STEER = 1;
N_STEER = 11;

STEER = [linspace(MIN_STEER, MAX_STEER, N_STEER)]';
D = [-1, 1]';

sons_node = [];
for i = 1:size(STEER, 1)
    for j = 1:size(D, 1)
        sons_node = [sons_node; calc_next_node(cur_node, STEER(i), D(j))];
    end
end

end

function son_node = calc_next_node(cur_node, steer, direction)

global goal_loc;
global Mmap;
global polar_footprint;

global max_cable_length;

arc_l = 3; % arc length
MOTION_RESOLUTION = 0.9;
SB_COST = 5;
STEER_COST = 1.0;
STEER_CHANGE_COST = 1.0;

D = [0:MOTION_RESOLUTION:arc_l]';

x = cur_node.x;
y = cur_node.y;
theta = cur_node.theta;

[x_list, y_list, yaw_list] = newMove(x, y, theta, direction, MOTION_RESOLUTION, steer, size(D, 1));

for i = 1:size(D, 1)

    x = x_list(i);
    y = y_list(i);
    theta = yaw_list(i);

    
    if ~isCollisionFree(Mmap, polar_footprint, x, y, theta)
        son_node = [];
        return ;
    end
    
end



if direction == cur_node.motion_direction
    added_cost = 0;
else
    added_cost = SB_COST;
end

% steer penalty
added_cost = added_cost + STEER_COST * abs(steer);

% steer change penalty
added_cost = added_cost + STEER_CHANGE_COST * abs(cur_node.steer - steer);

cost = cur_node.cost + added_cost + size(D, 1)*MOTION_RESOLUTION;

son_node = struct('index', -1, 'x', x_list(end), 'y', y_list(end), 'theta', yaw_list(end), ...
    's', [], 'phi', [], 'cost', cost, 'h', norm([x_list(end), y_list(end)]-goal_loc(1:2)), ...
    'hindex', [], 'steer', steer, 'motion_direction', direction, 'open', true, ...
    'fatherindex', cur_node.index, 'obs_vertices', [], 'mid_poses', [x_list, y_list, yaw_list]);

% We only need to verify the SEF of the last pose
old_s = cur_node.s;
son_node.s = polarRotateAndMoveToXy(polar_footprint(1, :), x_list(end), y_list(end), yaw_list(end));
son_node.obs_vertices = getCableState(cur_node.obs_vertices, old_s, son_node.s);
new_o = son_node.obs_vertices(end, :);
son_node.phi = atan2(new_o(2)-son_node.s(2), new_o(1)-son_node.s(1));

if ~isSEF(son_node)
    son_node = [];
    return ;
end

% max_cable_length
% We check the maximum cable length
temp = [son_node.obs_vertices; son_node.s];
if pathLength(temp) >= max_cable_length
    son_node = [];
    return ;
end

% We construct the hindex of the algorithm
cur_hindex = cur_node.hindex;
temp = [cur_node.x, cur_node.y; son_node.mid_poses(:, 1:2); son_node.x, son_node.y];
for i = 1:size(temp, 1) - 1
    cur_hindex = calSwing(cur_hindex, temp(i, :), temp(i+1, :));
end
son_node.hindex = cur_hindex;

end

function [X, Y, THETA] = newMove(x, y, theta, direction, MOTION_RESOLUTION, steer, n)
L = 3.0;
distance = direction*MOTION_RESOLUTION;
temp = wrapToPi(distance*tan(steer)/L);
i = [1:n]';
THETA = [theta; theta + i*temp];
X = zeros(n+1, 1);
Y = zeros(n+1, 1);
X(1) = x;
Y(1) = y;

for i = 2:n+1
    X(i) = X(i-1) + distance*cos(THETA(i-1));
    Y(i) = Y(i-1) + distance*sin(THETA(i-1));
end

X = X(2:end, :);
Y = Y(2:end, :);
THETA = THETA(2:end, :);

end
