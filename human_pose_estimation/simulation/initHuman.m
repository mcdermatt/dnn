%test script to set initial values for joint angles and velocities

j0pi = 0; %spine twist
j1pi = 0; % hips side
j2pi = 0; %bend over
j3pi = -50; %chicken wing
j4pi = 20; %butterfly
j5pi = 90; %curl
j6pi = -100; %elbow

j0vi = 0;
j1vi = 0;
j2vi = 0;
j3vi = 0;
j4vi = 0;
j5vi = 0;
j6vi = 0;

%"from workspace" block -> external force and torque block
% fx = timeseries((0:5)',[0 1 2 3 4]);
timevec = ((0:1000)/500)';
fz = timeseries(0*sin(timevec),timevec);
fx = timeseries(0*cos(timevec/2),timevec);
fy = timeseries(10*sin(5*timevec),timevec);
% fy = [0 0];
% fx = [0 0]; %Force on hand forwards

j0ll = -j0pi - 25;
j0ul = -j0pi + 25;
j1ll = -j1pi - 30;
j1ul = -j1pi + 30;
j2ll = -j2pi - 7.5;
j2ul = -j2pi + 60;
j3ll = -j3pi - 120; 
j3ul = -j3pi + 20;
j4ll = -j4pi - 30;
j4ul = -j4pi + 90;
j5ll = -j5pi - 180;
j5ul = -j5pi + 180;
j6ll = -j6pi -130;
j6ul = -j6pi;