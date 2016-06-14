function curvature_dot =  curvature_dot_comp(par_int,s,scale,v)
curvature_dot  = (par_int(2)/scale.^2 + 2*par_int(3).*s/scale.^3 + 3*par_int(4).*s.^2/scale^4).*v;
end