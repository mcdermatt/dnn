%Generates impedance ellipses for human arm in random configurations
%throughout workspace

%ASSUMPTIONS
%   Constant Human Parameters (same user for train and validation)
%   initial velocity = 0 for all joints
%   right handed user - IDEA: simplest case of this method can detect right
%       vs left handed users based on hand movements

%TODO
%   Make this a function that can be called to generate lots of data
%   O P T I M I Z E
%   cartesian distance from hand to shoulder OR joint angles
%       Lets do the 7 joint angles
%   Normalize all impedance values so that impedance of 1 can be used in
%       situations where no data is available for network
%   FIX PARFOR LOOP -> need to initialize variables outside of loop?


%Work flow for case of constant human
%1) set random initial joint angles
%2) get cartesian position of hand
%3) perturb hand of human with differential force in 6 directions, measure 
%   displacement to obtain virtual impedance 
%4) Save data on joint angles and endpoint impedance

beep off
numRuns = 24;
tic

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

    
    %TODO -> NEED TO SUBTRACT EACH DISPLACEMENT FROM A BASELINE
    fx = [0 1];
    fy = [0 0];
    fz = [0 0];
    simOut = sim('human7DOF.slx');
    axp = simOut.ax(end);
    fx = [0 -1];
    fy = [0 0];
    fz = [0 0];
    simOut = sim('human7DOF.slx');
    axn = simOut.ax(end);

    fx = [0 0];
    fy = [0 1];
    fz = [0 0];
    simOut = sim('human7DOF.slx');
    ayp = simOut.ay(end);
    fx = [0 0];
    fy = [0 -1];
    fz = [0 0];
    simOut = sim('human7DOF.slx');
    ayn = simOut.ay(end);

    fx = [0 0];
    fy = [0 0];
    fz = [0 1];
    simOut = sim('human7DOF.slx');
    azp = simOut.az(end);
    fx = [0 0];
    fy = [0 0];
    fz = [0 -1];
    simOut = sim('human7DOF.slx');
    azn = simOut.az(end);

    %append to total data
    A = [axp axn ayp ayn azp azn];
    data(i,:) = [pose A];
    i

end

csvwrite('data.mat', data)
toc

%tes = load('data.mat', '-ASCII');