function [x y]= drawtrajec_abcde(param,startpoint,endpoint,step)

a = param(1);
b = param(2);
c = -(85*param(1) - 108*param(3) + 27*param(4) - 4*param(5) + 22*param(2)*param(end))/(4*param(end)^2);

d = (9*(10*param(1) - 15*param(3) + 6*param(4) - param(5) + 2*param(2)*param(end)))/(2*param(end)^3);


e = -(9*(11*param(1) - 18*param(3) + 9*param(4) - 2*param(5) + 2*param(2)*param(end)))/(4*param(end)^4);

param= [ a ; b;c;d;e;param(end)];



%theta = repmat(startpoint(3),1,length(spacing)) + cubic_theta(param,spacing,1,startpoint(3));

%j = 1;
i = param(end)/step:param(end)/step:param(end);
%j = j+1;
x = integral_par_group_quart(0,[param(1:5)], i,0,1,startpoint(3)) + startpoint(1);
y  =  integral_par_group_quart(0,[param(1:5)], i,1,1,startpoint(3)) + startpoint(2);

x = [startpoint(1); x];
y = [startpoint(2); y];
end