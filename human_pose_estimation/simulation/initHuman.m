%TODO: Rename joints
%j0- hips x axis
%j1- hips y axis
%j2- hips z axis
%j3- shoulder x axis
%j4- shoulder y axis (chicken wing)
%j5- shoulder z axis (butterfly)
%j6- elbow

j0pi = 0; %initial hips side(?)
j1pi = 60; %60; %initial shoulder raise
j2pi = -45; %-45; %initial elbow

j0vi = 0;
j1vi = 0;
j2vi = 0;

%"from workspace" block -> external force and torque block
%[time force]??
% fx = [0 0]; %+x -> across body
% fx = timeseries((0:5)',[0 1 2 3 4]);
timevec = ((0:1000)/500)';
fz = timeseries(0*sin(timevec),timevec);
fx = timeseries(0*cos(timevec/2),timevec);
fy = timeseries(0*sin(timevec),timevec);
% fy = [0 0];
% fx = [0 0]; %Force on hand forwards

hipsll = -30;
hipsul = 35;
j0ll = -30;
j0ul = 30;
cwll = -90; %chicken wing lower limit
cwul = 20;
j1ll = -180;
j1ul = 180;
j2ll = 20;
j2ul = 100;