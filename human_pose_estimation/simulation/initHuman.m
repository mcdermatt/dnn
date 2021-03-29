j0pi = 0;
j1pi = 0;
j2pi = 0;

j0vi = 0;
j1vi = 0;
j2vi = 0;

%"from workspace" block -> external force and torque block
%[time force]??
% fx = [0 0]; %+x -> across body
% fx = timeseries((0:5)',[0 1 2 3 4]);
timevec = ((0:1000)/500)';
fz = timeseries(30*sin(timevec),timevec);
fx = timeseries(10*cos(timevec/2),timevec);
fy = timeseries(50*sin(timevec),timevec);
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
j2ll = -130;
j2ul = 8;