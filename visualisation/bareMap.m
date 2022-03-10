function bareMap()
global map_filename;

Mmap = imread(map_filename);
Mmap(1, :) = zeros(1, size(Mmap, 2));
Mmap(size(Mmap, 1), :) = zeros(1, size(Mmap, 2));
Mmap(:, 1) = zeros(size(Mmap, 1), 1);
Mmap(:, size(Mmap, 2)) = zeros(size(Mmap, 1), 1);

% For MATLAB visualization
imshow(Mmap');
xlabel('X')
ylabel('Y')
axis xy

end

