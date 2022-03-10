function waypoints = tracePath(V, index)

waypoints = [];
cur = V(index);
while 1
    if cur.fatherindex == -1
        waypoints = [cur; waypoints];
        break;
    end
    
    fatherindex = cur.fatherindex;
   waypoints = [cur; waypoints];
    cur = V(fatherindex);
end


end