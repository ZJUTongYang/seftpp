function initial_path = dijkstra_4dir(Mmap, start_loc, goal_loc)
% This function just reports an initial path for calculating initial cable
% state


queue = [start_loc];

temp_map = Mmap;

father_index = zeros(size(Mmap, 1), size(Mmap, 2), 2);
father_index(queue(1, 1), queue(1, 2), :) = [-1, -1];
cost = inf(size(Mmap, 1), size(Mmap, 2));
cost(queue(1, 1), queue(1, 2)) = 0;


while 1
    x = queue(1, 1);
    y = queue(1, 2);
    queue(1, :) = [];
    sons = [x, y] + [-1, 0; 0, -1; 0, 1; 1, 0];

    if x == goal_loc(1) && y == goal_loc(2)
        initial_path = [];
        cur = [x, y];
        while 1
            if cur(1) == start_loc(1) && cur(2) == start_loc(2)
                break;
            end
            father = father_index(cur(1), cur(2), :);
            initial_path = [cur(1), cur(2); initial_path];
            cur = father(1, 1, :);
        end
        break;
    end
    
    for i = 1:size(sons, 1)
        son = sons(i, :);
        
        if Mmap(son(1), son(2)) == 254
            continue;
        end
            
        if cost(son(1), son(2)) ~= inf
            % We are using BFS. 
            % If a node has been explored, then we cannot have a shortcut
            continue;
        end
        
        father_index(son(1), son(2), :) = [x, y];
        cost(son(1), son(2)) = cost(x, y) + 1;
        
        queue = [queue; son];
    end
    
    
end


end