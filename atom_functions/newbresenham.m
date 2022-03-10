function [X, Y] = newbresenham(x1, y1, x2, y2)

roundx1 = round(x1);
roundx2 = round(x2);
roundy1 = round(y1);
roundy2 = round(y2);

if roundx1 == roundx2
    if y1 <= y2
        P(:, 2) = [roundy1:roundy2]';
    else
        P(:, 2) = [roundy1:-1:roundy2]';
    end
    P(:, 1) = roundx1;
    
    X = P(:, 1);
    Y = P(:, 2);
    return ;
end

if roundy1 == roundy2
    if x1 <= x2
        P(:, 1) = [roundx1:roundx2]';
    else
        P(:, 1) = [roundx1:-1:roundx2]';
    end
    P(:, 2) = roundy1;
    
    X = P(:, 1);
    Y = P(:, 2);
    return ;
end

k = (y2 - y1)/(x2-x1);
b = y1 - k*x1;

if x1 < x2
    flag = 1;
    q = roundx1+0.5:roundx2-0.5;
else
    flag = -1;
    q = roundx1-0.5:-1:roundx2+0.5;
end

v = k.*q+b;
roundv = round(v);

temp = sign(k)*flag;

% calculate the size of P
diff = roundv(2:end-1) - roundv(1:end-2);
diff = floor(diff./temp)+1;
P = zeros(sum(diff), 2);

% insert data to P
current_start_index = 1;
for i = 1:size(v, 2)-1
    x = roundx1+i*flag;
    n = floor((roundv(i+1) - roundv(i))/temp)+1;
    P(current_start_index:current_start_index+n-1, 1) = x;
    P(current_start_index:current_start_index+n-1, 2) = [roundv(i):temp:roundv(i+1)]';
    current_start_index = current_start_index + n;
end

if roundy1 ~= roundv(1)
    P = [[repmat(roundx1, floor((roundv(1) - roundy1)/temp)+1, 1), [roundy1:temp:roundv(1)]']; P];
else
    P = [[roundx1, roundy1]; P];
end

if roundy2 ~= roundv(end)
    P = [P;[repmat(roundx2, floor((roundy2 - roundv(end))/temp)+1, 1), [roundv(end):temp:roundy2]']];
else
    P = [P; [roundx2, roundy2]];
end

X = P(:, 1);
Y = P(:, 2);
end