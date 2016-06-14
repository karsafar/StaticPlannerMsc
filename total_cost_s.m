function cost = total_cost_s(points, sf, dimension,children)
%http://planreg.towerhamlets.gov.uk/WAM/doc/401135-Page-24.pdf?extension=.pdf&page=24&id=401135&contentType=application/pdf&location=VOLUME4

%defining constraints
max_curv = 1/3.8;
% from diss-shuying
max_acc = 7;
max_dec = -7;

%if max curvature is exceeded an infinite cost should be given
%if a point is in collision an infinite cost should be given

%static costs
%point format [x;y;theta;curvature;curvature';speed;acceleration;jerk; time]
%centreline = 
%efficiency = stepsize*length(points);%total length
efficiency = sf;%total length
comfort = sum(abs(points(4,:))) + sum(abs(points(5,:)));
kinematics = (sum(abs(points(4,:))>max_curv))*1000000000000;
acceleration = (sum(((points(7,:))>max_acc)|((points(7,:))<max_dec)))*1000000000000;

behaviour = 0;
if~(isempty(children))
children_spread = ((children(:,end))<0.5)&((children(:,5))>0) & (sum(abs(children(:,1:4))>max_curv,2)==0);
temp_Array = 1:size(children,1);
children_no = (sum(children_spread));
children_span = max(temp_Array(children_spread))- min(temp_Array(children_spread))+1;
%robustness = -(children_no)*2-(children_span) + abs(points(4,end))*40;
%robustness = exp(-(children_no)/3)+ exp(-children_span/4) + abs(points(4,end))*40;
robustness = 400*exp(-(children_no)/3)+ 200*exp(-children_span/4) + abs(points(4,end))*40;
robustness = 400*exp(-(children_no)/3)+ 200*exp(-children_span/4) + abs(points(4,end))*100;
robustness = 400*exp(-(children_no)/3)+ 200*exp(-children_span/4) + abs(points(4,end))*50;
robustness = 400*exp(-(children_no)/3)+ 200*exp(-children_span/4) + abs(points(4,end))*200;

if(isempty(robustness))
    robustness = 0;
end
else
  robustness =0;
end
%constructing the decompostion of the vehile into circles
no_of_cirles =3;
radius = sqrt(dimension(2)^2/(4*no_of_cirles^2) + dimension(1)^2/4 );
d = 2*sqrt(radius^2 -dimension(1)^2/4);
centres = [d/2 0; d+d/2 0; 2*d+d/2 0]- [(dimension(2) - dimension(3))/2*ones(3,1) zeros(3,1) ];

%transformation = [cosd(position(3)) -sind(position(3)) position(1); sind(position(3)) cosd(position(3)) position(2); 0 0 1]*[ centres'; ones(1,3)];
% circle1 = [radius.*cos(theta) + transformation(1,1);radius.*sin(theta) + transformation(2,1)  ];
% circle2 = [radius.*cos(theta) + transformation(1,2);radius.*sin(theta) + transformation(2,2)  ];
% circle3 = [radius.*cos(theta) + transformation(1,3);radius.*sin(theta) + transformation(2,3)  ];
safety = 0;
weights = [1;10;10;10;10; 50];
weights = [0;0;0;0;1; 1];
weights = [0;0;0;0;1; 1];

%weights = [1000;0;0;0;0; 0];
%weights = [0;1000;0;0;0];

static_cost = [efficiency  comfort  behaviour  safety kinematics robustness]*weights;

%dynamic_cost
efficiency_d = points(9,end);%total time
energy_d = sum(points(6,:).^2);
comfort_d = sum(points(7,:).^2) + sum(points(8,:).^2) + sum(abs((points(6,:).^2).*points(4,:)));
comfort_d = 0.1*sum(points(7,:).^2) + 0.01*sum(points(8,:).^2) + sum(abs((points(6,:).^2).*points(4,:)));
%comfort_d = 0.11*sum(points(7,:).^2) + 0.015*sum(points(8,:).^2) + sum(abs((points(6,:).^2).*points(4,:)));
comfort_d = sum(abs((points(6,:).^2).*points(4,:)));
%comfort_d = sum(points(8,:).^2);
%comfort_d = sum(points(7,:).^2);

safety_d = 0 ;
weightsd = [10;1 ;0.1;0.1];
%weightsd = [1000;0 ;0;0];
%weightsd = [10;0 ;0;0];
weightsd = [100;0.1 ;1;0.1; 1];
%weightsd = [100;0.05 ;1;0.1];
%weightsd = [1000;0 ;0;0];
%weightsd = [10;10 ;10;0];
weightsd = [0;0 ;1; 0;1];
weightsd = [20;0 ;1; 0;1];
weightsd = [10;0 ;1; 0;1];
weightsd = [10;0 ;1; 0;1];

dynamic_cost= [efficiency_d energy_d comfort_d safety_d acceleration]*weightsd;

cost = static_cost + dynamic_cost;
end
