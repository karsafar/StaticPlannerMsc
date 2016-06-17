%// This is our initial conditon
m = zeros(10,5);
p1 = [1, 4];
p2 = [5, 5];

%// Draw a line from p1 to p2 on matrix m
x = p1(1):p2(1);
y = round((x - p1(1)) * (p2(2) - p1(2)) / (p2(1) - p1(1)) + p1(2));
m(sub2ind(size(m), y, x)) = 1