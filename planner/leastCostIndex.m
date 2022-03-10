function loc = leastCostIndex(V, Queue)

global goal_loc;

X = [V(Queue).x]';
Y = [V(Queue).y]';

hcost = sqrt((X-goal_loc(1)).^2 + (Y - goal_loc(2)).^2);
gcost = [V(Queue).cost]';
fcost = gcost + hcost;

loc = find(fcost == min(fcost));

end