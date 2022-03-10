function [son_x_index, son_y_index, son_theta_index] = calc_index(son, x_gridsize, y_gridsize, theta_gridsize)

son_x_index = floor(son.x / x_gridsize)+1;
son_y_index = floor(son.y / y_gridsize)+1;
son_theta_index = floor(wrapTo2Pi(son.theta) / theta_gridsize)+1;

end