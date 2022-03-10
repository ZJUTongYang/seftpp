function b = isCollisionFree(Mmap, polar_footprint, x, y, theta)

if x < 4 || x > size(Mmap, 1)-4 || y < 4 || y > size(Mmap, 1)-4
    b = false;
    return;
end

footprint = polarRotateAndMoveToXy(polar_footprint, x, y, theta); 
footprint = round(footprint);
for i = 1:size(footprint, 1)
    if Mmap(footprint(i, 1), footprint(i, 2)) ~= 0
        b = false;
        return ;
    end
end
b = true;


end