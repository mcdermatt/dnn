j0pi = 0; %spine twist
j1pi = 0; % hips side
j2pi = 0; %bend over
j3pi = 0; %chicken wing
j4pi = 20; %butterfly [-30 to 90]
j5pi = 60; %curl
j6pi = 30; %elbow

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
fz = timeseries(40*sin(timevec),timevec);
fx = timeseries(0*cos(timevec/2),timevec);
fy = timeseries(10*sin(timevec),timevec);
% fy = [0 0];
% fx = [0 0]; %Force on hand forwards

%TODO: Rename limits
hipsll = -30;
hipsul = 35;
j0ll = -30;
j0ul = 30;
cwll = -90; %chicken wing lower limit
cwul = 20;
j5ll = -180;
j5ul = 180;
j6ll = 0;
j6ul = 130;