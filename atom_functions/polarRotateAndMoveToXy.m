function [footprint, after_polar] = polarRotateAndMoveToXy(polar_footprint, x, y, theta)

after_polar = polar_footprint + [0, theta];

footprint = [after_polar(:, 1).*cos(after_polar(:, 2))+x, after_polar(:, 1).*sin(after_polar(:, 2))+y];

end