function resulting_path = planner_main(start_pose)

tic
[resulting_path, V, I] = planner(start_pose);
toc

if ~isempty(resulting_path)
    XY = [];
    for i = 1:size(resulting_path, 1)
        XY = [XY; resulting_path(i).mid_poses];
    end
    XY = XY(:, 1:2);

    showPose(resulting_path);
    plot(XY(:, 1), XY(:, 2), 'LineWidth', 2, 'Color', [90, 90, 165]/255);
end

end
