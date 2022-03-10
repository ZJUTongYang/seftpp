function A = findEqualHCNode(newH, V, indices)

if isempty(indices)
    return ;
end

for i = 1:size(indices, 1)
    oldH = V(indices(i)).hindex;
    if size(oldH, 1) == size(newH, 1) && norm(oldH - newH) < 0.1
        A = indices(i);
        return ;
    end
end

end