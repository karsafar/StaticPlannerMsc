
function curvature = curvature_comp(par_int,s,scale)
curvature = par_int(1)/scale + par_int(2).*s/scale^2 + par_int(3).*s.^2/scale^3 + par_int(4).*s.^3/scale^4 ;
end