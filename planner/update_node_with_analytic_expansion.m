function [is_updated, final_path] = update_node_with_analytic_expansion(cur_node)
global goal_loc;
if norm([goal_loc(1)-cur_node.x, goal_loc(2)-cur_node.y]) < 2
    is_updated = true;
else
    is_updated = false;
end
final_path = [];


end


