function showBasePoint(x, y)
hold on
P = [x, y] + 2*[0, 1; -sqrt(3)/2, -0.5; sqrt(3)/2, -0.5];
patch(P(:, 1), P(:, 2), [165, 90, 90]/255);

end