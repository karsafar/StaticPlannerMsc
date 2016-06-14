function [s,T, a, jerk, v, remainder] = velgen_draw(v0,vf,a0,af,sf,step,remainder)
%speed profile generation



%velocity profile v = p0 + p1*t + p2* t^2 + p3* t^3
a = (a0-af)/12;
b = (v0 + vf)/2;
c = -sf;
temp = (b^2-4*a*c);
if(round(a,-5) == 0)
    T = -c/b;
elseif( temp>=0)
    T = (-b + sqrt(temp))/(2*a); % should yo consider also negative sqrt?
else
    T =nan;
end

p0 = v0;
p1 = a0;
p2 = (3*(vf-v0)-(af+2*a0)*T)/T^2;
p3 = (-2*(vf-v0)+(af+a0)*T)/T^3;
t= 0:step:T;
if(isempty(t))
    tttopop = 1;
end
t= step-remainder:step:T;
v = p0 + p1.*t + p2.*t.^2 + p3.*t.^3;
a = p1 + 2*p2.*t + 3*p3.*t.^2;
jerk = 2*p2 + 6*p3.*t;
s =  p0.*t + p1.*t.^2/2 + p2.*t.^3/3 + p3.*t.^4/4;
remainder = T - t(end);
end