function [Mmap, critical_points] = generatePolyMap(map_filename)
% map_filename: -
% Mmap: the map array
% critical_points: The location of all obstacle vertices

%% We prepare the map
tic

Mmap = imread(map_filename);
Mmap(1, :) = zeros(1, size(Mmap, 2));
Mmap(size(Mmap, 1), :) = zeros(1, size(Mmap, 2));
Mmap(:, 1) = zeros(size(Mmap, 1), 1);
Mmap(:, size(Mmap, 2)) = zeros(size(Mmap, 1), 1);

% For MATLAB visualization
imshow(Mmap');
xlabel('X')
ylabel('Y')
axis xy

% transform the map to an occupancy grid
Mmap = int16(Mmap);
for i= 1:size(Mmap, 1)
    for j = 1:size(Mmap, 2)
        if Mmap(i, j) == 0
            Mmap(i, j) = 254;
        elseif Mmap(i, j) >= 254
            Mmap(i, j) = 0;
        end
    end
end

Mmap(1, :) = 254;
Mmap(end, :) = 254;
Mmap(:, 1) = 254;
Mmap(:, end) = 254;

% We identify the topology of obstacles
global g_obs;
g_obs = sedFill(Mmap);

%% We prepare the critical obstacles (grids)
% For each obstacle, we check whether its 8-dir neighbour is a
% 4-neighbour obstacle-free grid
[X, Y] = find(Mmap == 254);
all_temp = [X+1, Y+1; X-1, Y-1; X+1, Y-1; X-1, Y+1];
all_temp(all_temp(:, 1) <= 1 | all_temp(:, 1) >= size(Mmap, 1) | all_temp(:, 2) <= 1 | all_temp(:, 2) >= size(Mmap, 2), :) = [];

for i = size(all_temp, 1):-1:1
    x = all_temp(i, 1);
    y = all_temp(i, 2);
    if Mmap(x-1, y) == 254 || Mmap(x+1, y) == 254 || Mmap(x, y-1) == 254 ||Mmap(x, y+1) == 254
        all_temp(i, :) = [];
    end
end

% We use precise location of obstacle corners
critical_points = [];
for i = 1:size(all_temp, 1)
    x = all_temp(i, 1);
    y = all_temp(i, 2);
    if Mmap(x-1, y-1) == 254
        critical_points = [critical_points; [x-0.5, y-0.5]];
    elseif Mmap(x-1, y+1) == 254
        critical_points = [critical_points; [x-0.5, y+0.5]];
    elseif Mmap(x+1, y-1) == 254
        critical_points = [critical_points; [x+0.5, y-0.5]];
    else
        critical_points = [critical_points; [x+0.5, y+0.5]];
    end
end

% critical_points = all_temp;
hold on
plot(critical_points(:, 1), critical_points(:, 2), '*')

end



