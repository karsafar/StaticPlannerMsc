function [no_of_coll positions] = collision_detection(obstacles,traj, dimension)


%% what is that
no_of_circles = 3;
radius = sqrt(dimension(2)^2/(4*no_of_circles^2) + dimension(1)^2/4 );
d = 2*sqrt(radius^2 -dimension(1)^2/4);
centres = [d/2 0; d+d/2 0; 2*d+d/2 0]- [(dimension(2) - dimension(3))/2*ones(3,1) zeros(3,1) ];


%% what is this transforamtion
%transformation = [cosd(position(3)) -sind(position(3)) position(1); sind(position(3)) cosd(position(3)) position(2); 0 0 1]*[ centres'; ones(1,3)];
transformation_x = cos(repmat(traj(3,:)',1,no_of_circles)).*repmat(centres(:,1)',size(traj,2),1)-sin(repmat(traj(3,:)',1,no_of_circles)).*repmat(centres(:,2)',size(traj,2),1) + repmat(traj(1,:)',1,no_of_circles);
  transformation_y =   sin(repmat(traj(3,:)',1,no_of_circles)).*repmat(centres(:,1)',size(traj,2),1)-cos(repmat(traj(3,:)',1,no_of_circles)).*repmat(centres(:,2)',size(traj,2),1) + repmat(traj(2,:)',1,no_of_circles);
transformation = [transformation_x(:)' ; transformation_y(:)' ];
% circle1 = [radius.*cos(theta) + transformation(1,1);radius.*sin(theta) + transformation(2,1)  ];
% circle2 = [radius.*cos(theta) + transformation(1,2);radius.*sin(theta) + transformation(2,2)  ];
% circle3 = [radius.*cos(theta) + transformation(1,3);radius.*sin(theta) + transformation(2,3)  ];

%% what is this
replicax_car = repmat(transformation(1,:), size(obstacles,2),1);%to check if samples in points and centreline are equal
replicax_obstacles_l = repmat(obstacles(1,:)',1,size(transformation,2));
replicax_obstacles_r = repmat(obstacles(3,:)',1,size(transformation,2));

replicay_car = repmat(transformation(2,:), size(obstacles,2),1);
replicay_obstacles_l = repmat(obstacles(2,:)',1,size(transformation,2));
replicay_obstacles_r = repmat(obstacles(4,:)',1,size(transformation,2));


dist_to_obst_l= sqrt((replicax_car-replicax_obstacles_l).^2 + (replicay_car-replicay_obstacles_l).^2)< (radius + 0.3);
dist_to_obst_r= sqrt((replicax_car-replicax_obstacles_r).^2 + (replicay_car-replicay_obstacles_r).^2)< (radius + 0.3);
if(any(any(dist_to_obst_l))~=0)&&(any(any(dist_to_obst_r))==0)
    positions_x = replicax_obstacles_l(dist_to_obst_l);
    positions_y = replicay_obstacles_l(dist_to_obst_l);
    no_of_coll_matr= dist_to_obst_l(:,1:length(transformation(1,:))/no_of_circles)...
        |dist_to_obst_l(:,length(transformation(1,:))/no_of_circles +1 :length(transformation(1,:))*2/no_of_circles)...
        |dist_to_obst_l(:,length(transformation(1,:))*2/no_of_circles+1:length(transformation(1,:)));
    no_of_coll = sum(any(no_of_coll_matr));
elseif(any(any(dist_to_obst_l))==0)&&(any(any(dist_to_obst_r))~=0)
    positions_x = replicax_obstacles_r(dist_to_obst_r);
    positions_y = replicay_obstacles_r(dist_to_obst_r);
    no_of_coll_matr= dist_to_obst_r(:,1:length(transformation(1,:))/no_of_circles)...
        |dist_to_obst_r(:,length(transformation(1,:))/no_of_circles +1 :length(transformation(1,:))*2/no_of_circles)...
        |dist_to_obst_r(:,length(transformation(1,:))*2/no_of_circles+1:length(transformation(1,:)));
    no_of_coll = sum(any(no_of_coll_matr));
elseif (any(any(dist_to_obst_l))~=0)&&(any(any(dist_to_obst_r))~=0)
    positions_x = replicax_obstacles(dist_to_obst_r);
    positions_y = replicay_obstacles(dist_to_obst_r);
    positions_x = [positions_x replicax_obstacles(dist_to_obst_l)];
    positions_y = [positions_y replicay_obstacles(dist_to_obst_l)];
    
    no_of_coll_matr_l= dist_to_obst_l(:,1:length(transformation(1,:))/no_of_circles)...
        |dist_to_obst_l(:,length(transformation(1,:))/no_of_circles +1 :length(transformation(1,:))*2/no_of_circles)...
        |dist_to_obst_l(:,length(transformation(1,:))*2/no_of_circles+1:length(transformation(1,:)));
      no_of_coll_matr_r= dist_to_obst_r(:,1:length(transformation(1,:))/no_of_circles)...
        |dist_to_obst_r(:,length(transformation(1,:))/no_of_circles +1 :length(transformation(1,:))*2/no_of_circles)...
        |dist_to_obst_r(:,length(transformation(1,:))*2/no_of_circles+1:length(transformation(1,:)));
    no_of_coll = sum(any(no_of_coll_matr_l|no_of_coll_matr_r));
else
    positions_x =[];
    positions_y = [];
    no_of_coll = 0;
end

positions = unique([positions_x positions_y],'rows','stable');
%no of collisions

%behaviour = 0;

end
