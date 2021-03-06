function no_coll = check_path(optimal,obstacles, lattice, start_car, car_pose)


startpose = start_car.start;
param_start = start_car.traj;

pos_x = lattice.x;
pos_y = lattice.y;
orientation = lattice.theta;

param_lattice = lattice.param ;

sub_sample = lattice.sample;
dimension = start_car.dimension;




[x,y]= drawtrajec_abcde(param_start(optimal(2,2), 1 :6)' ,startpose(1:4),[],sub_sample);
param = param_start(optimal(2,2),1:6);
a = param(1);
b = param(2);
c = -(85*param(1) - 108*param(3) + 27*param(4) - 4*param(5) + 22*param(2)*param(end))/(4*param(end)^2);
d = (9*(10*param(1) - 15*param(3) + 6*param(4) - param(5) + 2*param(2)*param(end)))/(2*param(end)^3);

e = -(9*(11*param(1) - 18*param(3) + 9*param(4) - 2*param(5) + 2*param(2)*param(end)))/(4*param(end)^4);
param_abcde = [ a ; b;c;d;e];
theta = quartic_theta(param_abcde,0:param(end)/sub_sample:param(end),1,startpose(3));
no_coll_overall =[];
coll_pos_overall=[];
%output variables collision no per section and obstacles per section;
%% collision check for the initial trajectories
% [no_coll_sec, coll_pos] = collision_detection(obstacles(:,1:sub_sample+1),[x';y';theta], dimension);

[no_coll_sec, coll_pos] = static_obstacles(obstacles(:,1:sub_sample+1),[x';y';theta], dimension,car_pose);

no_coll_overall = [ no_coll_overall no_coll_sec];
coll_pos_overall = [coll_pos_overall;coll_pos];
%% collision check for all other sections
for i = 2: length(optimal)-1
    startpoint = [pos_x(optimal(2,i),i-1); pos_y(optimal(2,i),i-1); orientation(optimal(2,i),i-1)];
    endpoint =[];
    [x ,y ,theta1 ,curvature1 ,spacing] = drawtrajec_abcd(param_lattice(optimal(2,i+1),(i-2)*7 +1 :(i-2)*7 +5, optimal(2,i))' ,startpoint,endpoint,sub_sample);
%     [no_coll_sec, coll_pos] = collision_detection(obstacles(:,(i-1)*sub_sample+1:(i-1)*sub_sample+1 +sub_sample +1) ,[x';y';theta1], dimension);

    %% stick somewhere here the static obstacles code
    [no_coll_sec, coll_pos] = static_obstacles(obstacles(:,(i-1)*sub_sample+1:(i-1)*sub_sample+1 +sub_sample +1) ,[x';y';theta1],dimension, car_pose);
    
    no_coll_overall = [ no_coll_overall no_coll_sec];
    coll_pos_overall = [coll_pos_overall; coll_pos];
end

no_coll= sum(no_coll_overall);
end
