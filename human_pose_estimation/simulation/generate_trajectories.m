%generates dataset with trajectories x(t), y(t), z(t) and joint angles for
%last position in trajectory

%Arm moves do to RANDOM FORCES acting on hand

%ideally this should encode information on virtual inertia and thus we will
%not need to calculate endpoint impedance

beep off
numRuns = 1;
trajPts = 10; %number of points in each trajectory
fixedStepSize = 0.0001; %from simulink solver

tic

traj = [];

data = []; %[pose A]
%run a few times
% parpool(3)
% parfor i = 1:numRuns %parallel for loop
for i = 1:numRuns
    %get random joint angles within limits
    j0pi = rand()*20-10;
    j1pi = rand()*20 - 10;
    j2pi = rand()*10 - 5;
    j3pi = rand()*110 - 90;
    j4pi = rand()*120 - 30;
    j5pi = rand()*120 -30;
    j6pi = -rand()*130;

    pose = [j0pi j1pi j2pi j3pi j4pi j5pi j6pi];

    %update joint limits
    j0ll = -j0pi - 25;
    j0ul = -j0pi + 25;
    j1ll = -j1pi - 30;
    j1ul = -j1pi + 30;
    j2ll = -j2pi - 7.5;
    j2ul = -j2pi + 60;
    j3ll = -j3pi - 90; 
    j3ul = -j3pi + 20;
    j4ll = -j4pi - 30;
    j4ul = -j4pi + 90;
    j5ll = -j5pi - 180;
    j5ul = -j5pi + 180;
    j6ll = -j6pi - 130;
    j6ul = -j6pi;

    j0vi = 0;
    j1vi = 0;
    j2vi = 0;
    j3vi = 0;
    j4vi = 0;
    j5vi = 0;
    j6vi = 0;
    
    %case of constant cartesian external forces (no gravity)
    fx = [0 0.1*randn()];
    fy = [0 0.1*randn()];
    fz = [0 0.1*randn()];
    simOut = sim('human7DOF.slx');
    
    startPos = [simOut.x(1) simOut.y(1) simOut.z(1)];
    
    %get array of xyz points in trajectory
    for j = 1:trajPts
        s = j*floor(length(simOut.x)/trajPts);
        data(j,:) = [simOut.x(s) simOut.y(s) simOut.z(s)] - startPos;   
    end
    
    %get joint angles at final point
    jointPos = [simOut.j0pf(s) simOut.j1pf(s) simOut.j2pf(s) simOut.j3pf(s) ...
        simOut.j4pf(s) simOut.j5pf(s) simOut.j6pf(s)];
    
end

csvwrite('traj_data.mat', data)
toc

%tes = load('data.mat', '-ASCII');