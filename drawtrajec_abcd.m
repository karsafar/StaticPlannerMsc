function [x y theta curvature spacing]= drawtrajec_abcd(param,startpoint,endpoint,step)

a=param(1);
b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
param= [ a ; b;c;d;param(end)];


spacing = 0:param(end)/step:param(end);
%theta = repmat(startpoint(3),1,length(spacing)) + cubic_theta(param,spacing,1,startpoint(3));
theta =  cubic_theta(param,spacing,1,startpoint(3));
curvature = curvature_comp(param,spacing,1);
%j = 1;
i = param(end)/step:param(end)/step:param(end);
%j = j+1;
x = integral_par_group(0,[param(1:4)], i,0,1,startpoint(3)) + startpoint(1); 
y =  integral_par_group(0,[param(1:4)], i,1,1,startpoint(3)) + startpoint(2);

x = [startpoint(1); x];
y = [startpoint(2); y];
end