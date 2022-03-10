function [resulting_path, V, I] = planner(start_pose)

global Mmap;

V = start_pose;
xnum = size(Mmap, 1);
ynum = size(Mmap, 2);
thetanum = 60;

x_gridsize = size(Mmap, 1) / xnum;
y_gridsize = size(Mmap, 2) / ynum;
theta_gridsize = 2 * pi / thetanum;

I(xnum+1, ynum+1, thetanum+1).indices = [];

Queue = [1];

resulting_path = [];

while 1
    if isempty(Queue)
        break;
    end
    
    % We find the least-cost motion
    cur_in_queue_index = leastCostIndex(V, Queue);
    cur_in_queue_index = cur_in_queue_index(1);
    cur_index = Queue(cur_in_queue_index, 1);
    Queue(cur_in_queue_index, :) = [];
    
    V(cur_index, 1).open = false;
    
    cur_node = V(cur_index, 1);
    
    [is_updated, final_path] = update_node_with_analytic_expansion(cur_node);
    
    if is_updated
        fprintf("path found\n");
        fprintf("number of nodes: %d\n", size(V, 1))
        resulting_path = tracePath(V, cur_node.index);
        return ;
    end
    
    % Here we calculate all elements of sons,
    % except their index, because we haven't sure they will be added into the storage
    sons = get_neighbors(cur_node);
    
    for i = 1:size(sons)
        son = sons(i);
        
        % Here the Hindex is the h signature of the cable AND THE ROBOT'S
        % CENTER, or else different cable states do not share a common end
        % newHindex = calSwing(cur_node.hindex, [cur_node.x, cur_node.y, cur_node.theta], [son.x, son.y, son.theta]);
        newHindex = son.hindex;
        
        newcost = son.cost;
        
        % find the index of the son
        newsonindex = size(V, 1) + 1;
        son.index = newsonindex;
        
        [son_x_index, son_y_index, son_theta_index] = calc_index(son, x_gridsize, y_gridsize, theta_gridsize);
        
        if isempty(I(son_x_index, son_y_index, son_theta_index).indices) || ...
                ~isEqualHC(newHindex, V, I(son_x_index, son_y_index, son_theta_index).indices)
            % If the voxel hasn't been visited
            % If we find non-homotopic paths to the same son voxel, we
            % preserve them both
            Queue = [Queue; newsonindex];
            V = [V; son];
            I(son_x_index, son_y_index, son_theta_index).indices = ...
                [I(son_x_index, son_y_index, son_theta_index).indices; newsonindex];
            
        else
            % We can find the equivalent Hindex in the I matrix, but we may
            % carry out a shortcut
            oldindex = findEqualHCNode(newHindex, V, I(son_x_index, son_y_index, son_theta_index).indices);
            
            if V(oldindex).cost <= newcost
                continue;
            end
            
            % If the old node is open, then it is in the queue, we do
            % nothing, because the cost has been updated in the storage
            if V(oldindex).open == false
                Queue = [Queue; oldindex];
            end
            
            V(oldindex) = son;
            V(oldindex).index = oldindex;
            
        end
        
    end
    
end % main while

fprintf("Fail to find a path. We return.\n");
resulting_path = [];


end
