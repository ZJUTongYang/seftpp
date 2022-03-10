function b = isEqualHC(newH, Storage, indices)

b = false;
for i = 1:size(indices, 1)
    oldH = Storage(indices(i)).hindex;
    if size(oldH, 1) == size(newH, 1) && norm(oldH - newH) < 0.1
        b = true;
        return ;
    end
end

end