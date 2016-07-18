clear all
XWorldLimits = [0 10];
YWorldLimits = [0 10];
GridSize = [40 10];

% generate simple occupancy grid
OccupGrid = zeros(GridSize);
for i = 6:13
    for j = 2:3
        OccupGrid(i,j) = 1;
    end
end
figure(1)
imshow(OccupGrid)

% generate a matrix of a line

LineGrid = zeros(GridSize);
p1 = [1, 6];
p2 = [10, 30];

% Draw a line from p1 to p2 on matrix 
x = p1(1):p2(1);
y = round((x - p1(1)) * (p2(2) - p1(2)) / (p2(1) - p1(1)) + p1(2));
LineGrid(sub2ind(size(LineGrid), y, x)) = 1;
figure(2)
imshow(LineGrid);
any(any(OccupGrid.*LineGrid))