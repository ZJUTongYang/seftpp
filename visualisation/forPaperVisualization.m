close all

for num = 1:size(solution_paths, 1)
   if isempty(solution_paths{num})
       continue;
   end
   
   h = figure(num);
   bareMap;
   hold on
   
   
   showBasePoint(ran_base_point(num, 1), ran_base_point(num, 2))
   
   
   
   resulting_path = solution_paths{num};
   
   XY = [];
   for i = 1:size(resulting_path, 1)
       XY = [XY; resulting_path(i).mid_poses];
   end
   XY = XY(:, 1:2);
   pathLength(XY)
   
   %   showPose(resulting_path);
   plot(XY(:, 1), XY(:, 2), 'LineWidth', 3, 'Color', [90, 90, 165]/255);
   
   set(gcf, 'position', [0, 0, 2000, 1000])
   
   
    showPose(resulting_path(1), [199, 120, 120]);
    showPose(resulting_path(end), [120, 120, 199]);
    
   for i = 2:size(resulting_path, 1)-1
       pre_dir = atan2(resulting_path(i).y-resulting_path(i-1).y, resulting_path(i).x-resulting_path(i-1).x);
       aft_dir = atan2(resulting_path(i+1).y-resulting_path(i).y, resulting_path(i+1).x-resulting_path(i).x);
       if abs(wrapToPi(aft_dir - pre_dir)) > pi/2
           showPose(resulting_path(i));
       end
   end

   temp = getframe(h);
%    imwrite(temp.cdata, ['case', num2str(num), '.png']);
   
  
   
end