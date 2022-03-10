function showPose(poses, color)

global polar_footprint;

hold on
for i = 1:size(poses, 1)
    thePose = poses(i);
    temp_polar_footprint = [polar_footprint; polar_footprint(1, :)];
    
    footprint = polarRotateAndMoveToXy(temp_polar_footprint, thePose.x, thePose.y, thePose.theta);
    
    if nargin == 1
        color = [0, 0, 0];
    end
    
    plot(footprint(:, 1), footprint(:, 2), 'LineWidth', 1.5, 'Color', color/255);
    
    tether = [thePose.obs_vertices; footprint(1, :)];
    plot(tether(:, 1), tether(:, 2), 'LineWidth', 1.5, 'Color', [165, 165, 165]/255);

end

end