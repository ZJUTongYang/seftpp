function L = pathLength(thepath)

if isempty(path) || size(thepath, 2) ~= 2
    warning('check here.\n');
    L = inf;
    return ;
end

L = 0;
for i = 1:size(thepath, 1)-1
    L = L + norm(thepath(i+1, :)-thepath(i, :));
end

end