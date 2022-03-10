function result_obs_list = getCableState(obs_list, olds, news)
% olds, news: the locaion of the tether-robot contact point
% We assume that the motion is a primitive, so a straight motion of s is
% assumed.

global Mmap;
global critical_points;

tri = [obs_list(end, :); olds; news];

temp_critical_points = critical_points;
% avoiding insert the last obstacle again
temp = temp_critical_points(:, 1) == tri(1, 1);
if any(temp)
    temp = temp & temp_critical_points(:, 2) == tri(1, 2);
    if any(temp)
        loc = find(temp);
        temp_critical_points(loc, :) = [-1, -1];
    end
end


[in, on] = inpolygon(temp_critical_points(:, 1), temp_critical_points(:, 2), ...
    tri(:, 1), tri(:, 2));

if ~any(in) && ~any(on)
    result_obs_list = tryRemoveObstacles(obs_list, news);
else
    pre_theta = atan2(tri(2, 2)-tri(1, 2), tri(2, 1)-tri(1, 1));
    corners = temp_critical_points(in, :);
    
    % we find the corner with least angle diff
    diff = corners - tri(1, :);
    theta = atan2(diff(:, 2), diff(:, 1));
    temp_theta = abs(wrapToPi(theta - pre_theta));
    corner = corners(temp_theta == min(temp_theta), :);
    % If there are multiple least-angle-diff corners, we choose the nearest
    % one
    diff = corner - tri(1, :);
    dis = sqrt(sum(diff.^2, 2));
    [~, order] = sort(dis);
    corner = corner(order, :);
    
    result_obs_list = [obs_list; corner];
    
    result_obs_list = getCableState(result_obs_list, olds, news);
end

end


function result_obs_list = tryRemoveObstacles(obs_list, news)

global critical_points;
global Mmap;

result_obs_list = obs_list;

if size(obs_list, 1) < 2
    return ;
end

tri = [obs_list(end-1:end, :); news];
[X, Y] = newbresenham(tri(1, 1), tri(1, 2), tri(3, 1), tri(3, 2));
P = [X, Y];
for i = 2:size(P, 1)
    if Mmap(P(i, 1), P(i, 2)) ~= 0
        return ;
    end
end

temp_critical_points = critical_points;

% avoiding insert the obstacle again
temp = temp_critical_points(:, 1) == tri(1, 1);
if any(temp)
    temp = temp & temp_critical_points(:, 2) == tri(1, 2);
    if any(temp)
        loc = find(temp);
        temp_critical_points(loc, :) = [-1, -1];
    end
end
temp = temp_critical_points(:, 1) == tri(2, 1);
if any(temp)
    temp = temp & temp_critical_points(:, 2) == tri(2, 2);
    if any(temp)
        loc = find(temp);
        temp_critical_points(loc, :) = [-1, -1];
    end
end

[in, on] = inpolygon(temp_critical_points(:, 1), temp_critical_points(:, 2), ...
    tri(:, 1), tri(:, 2));

if ~any(in)
    result_obs_list = obs_list(1:end-1, :);
    result_obs_list = tryRemoveObstacles(result_obs_list, news);
end

end