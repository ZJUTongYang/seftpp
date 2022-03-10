function ObsPosition = sedFill(S)
% S: the input 2D map, with obstacles being labelled as -1
% ObsPosition: n*2 array of obstacle positions, each corresponds to one
% connected obstacle

ObsPosition = [];

%% We remove the boundary obstacles
mask = S;

seed = [1, 1];
queue = seed;
while ~isempty(queue)
    cur = queue(end, :);
    if cur(1) >= 1 && cur(2) >= 1 && cur(1) <= size(mask, 1) && cur(2) <= size(mask, 2) && ...
            mask(cur(1), cur(2)) >= 254        
        mask(cur(1), cur(2)) = 0;
        queue = [queue(1:end-1, :);
            cur(1)-1, cur(2);
            cur(1)+1, cur(2);
            cur(1), cur(2)-1;
            cur(1), cur(2)+1];
    else
        queue(end, :) = [];
    end
end

%% We identify obstacles
for i = 2:size(mask, 1)-1
    for j = 2:size(mask, 2)-1
        if mask(i, j) < 254
            continue;
        end
        
        seed = [i, j];
        ObsPosition = [ObsPosition; seed];
        % We start a sedfill to eliminate all connected obstacles
        queue = seed;
        while ~isempty(queue)
            cur = queue(end, :);
            if mask(cur(1), cur(2)) >= 254
                
                mask(cur(1), cur(2)) = 0;
                queue = [queue(1:end-1, :);
                            cur(1)-1, cur(2); 
                            cur(1)+1, cur(2);
                            cur(1), cur(2)-1;
                            cur(1), cur(2)+1];
            else
                queue(end, :) = [];
            end
        end
    end
end





