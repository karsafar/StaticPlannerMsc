function draw_car_obstacle(dimension,position)


%      rectangle = [ 0 0; dimension(1) 0; dimension(1) dimension(2); 0 dimension(2)]- [dimension(1)/2*ones(4,1) dimension(2)/2*ones(4,1)];
%      front = [ 0 dimension(2)/2];
%      transformation = [cosd(position(3)) -sind(position(3)) position(1); sind(position(3)) cosd(position(3)) position(2); 0 0 1]*[rectangle' front'; ones(1,5)];
%      plot([transformation(1,1:end-1) transformation(1,1)],[transformation(2,1:end-1) transformation(2,1)]);
%      hold on
%      plot(transformation(1,end), transformation(2,end),'r*');

%the origin is placed on the middle of the rear wheels
rectangle = [ 0 0; dimension(2) 0; dimension(2) dimension(1); 0 dimension(1)]- [(dimension(2) - dimension(3))/2*ones(4,1) dimension(1)/2*ones(4,1) ];
front = [(dimension(2) + dimension(3))/2 0];
%draw circles 
% no_of_cirles =3;
% radius = sqrt(dimension(2)^2/(4*no_of_cirles^2) + dimension(1)^2/4 );
% d = 2*sqrt(radius^2 -dimension(1)^2/4);
% centres = [d/2 0; d+d/2 0; 2*d+d/2 0]- [(dimension(2) - dimension(3))/2*ones(3,1) zeros(3,1) ];





% rotation counter-clockwise about the origin
transformation = [cosd(position(3)) -sind(position(3)) position(1); sind(position(3)) cosd(position(3)) position(2); 0 0 1]*[rectangle' front'; ones(1,5)];
h = fill([transformation(1,1:end-1) transformation(1,1)],[transformation(2,1:end-1) transformation(2,1)],'r');

set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

 hold on
h = plot(transformation(1,end), transformation(2,end),'r*');

 set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');


% transformation = [cosd(position(3)) -sind(position(3)) position(1); sind(position(3)) cosd(position(3)) position(2); 0 0 1]*[rectangle' front' centres'; ones(1,8)];
% plot([transformation(1,1:4) transformation(1,1)],[transformation(2,1:4) transformation(2,1)]);
% hold on
% plot(transformation(1,5:end), transformation(2,5:end),'r*');
% theta = 0:0.001:2*pi;
% 
% circle1 = [radius.*cos(theta) + transformation(1,6);radius.*sin(theta) + transformation(2,6)  ];
% circle2 = [radius.*cos(theta) + transformation(1,7);radius.*sin(theta) + transformation(2,7)  ];
% circle3 = [radius.*cos(theta) + transformation(1,8);radius.*sin(theta) + transformation(2,8)  ];
% plot(circle1(1,:),circle1(2,:));
% plot(circle2(1,:),circle2(2,:));
% plot(circle3(1,:),circle3(2,:));




end