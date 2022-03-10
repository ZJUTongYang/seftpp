function newH = calSwing(oldH, oldp, newp)

global g_obs;

% We have assumed that no two g_axis are coincident. Hence we check whether
% the new motion primitive has gone across some axis

newH = oldH;
for i = 1:size(g_obs, 1)
    if oldp(2) >= g_obs(i, 2) && newp(2) >= g_obs(i, 2) 
        if oldp(1) > g_obs(i, 1)+0.5 && newp(1) < g_obs(i, 1)+0.5
            if ~isempty(oldH) && oldH(end) == i
                newH = oldH(1:end-1, :);
            else
                newH = [oldH; -i];
            end
            return ;
        elseif oldp(1) < g_obs(i, 1)+0.5 && newp(1) > g_obs(i, 1)+0.5
            if ~isempty(oldH) && oldH(end) == -i
                newH = oldH(1:end-1, :);
            else
                newH = [oldH; i];
            end
            return ;
        end
    end
end



end