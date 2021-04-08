%generates dataset with trajectories x(t), y(t), z(t) and joint angles for
%last position in trajectory

%Arm moves due to RANDOM FORCES of CONSTANT MAGNITUDE AND DIRECTION acting on hand

%ideally this should encode information on virtual inertia and thus we will
%not need to calculate endpoint impedance directly

%Make sure to solve with LINEAR TIMESTEPS!!!
%   Adaptive solver is unrealistic because it provides extra data in 
%      critical areas of trajectory
%   Models trained in adaptive solver don't seem to work with input from
%   linear solver (which makes sense)

beep off
numTraj = 10000;
trajPerChunk = 100;
trajPts = 10; %number of points in each trajectory
% trajTotal = [];
trajTotal = zeros(trajPts,6,numTraj);
% jointPosTotal = [];
jointPosTotal = zeros(numTraj,7);

%NOTE: This works WAAAYY faster withtout visual simulation on 

tic
count = 0;

% parpool(3)
% parfor m = 1:(floor(numTraj/trajPerChunk)) %parallel for loop

m = 1;
while m <= (floor(numTraj/trajPerChunk))

        traj = zeros(trajPts, 3, trajPerChunk);
        trajAngs = zeros(trajPts, 3, trajPerChunk);
        jointPos = zeros(trajPerChunk, 7);    

        %run a few times
        i = 1;
    while i <= trajPerChunk
        try
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
            mult = 1;
            fx = [0 mult*randn()];
            fy = [0 mult*randn()];
            fz = [0 mult*randn()];
            simOut = sim('human7DOF.slx');

            startPos = [simOut.x(1) simOut.y(1) simOut.z(1)];

            %get array of xyz points in trajectory
            for j = 1:trajPts
                s = j*floor(length(simOut.x)/trajPts);
                traj(j,:,i) = [simOut.x(s) simOut.y(s) simOut.z(s)] - startPos;
                trajAngs(j,:,i) = [simOut.ang(s,1) simOut.ang(s,2) simOut.ang(s,3)];
            end

            %get joint angles at final point
            jointPos(i,:) = [simOut.j0pf(s) simOut.j1pf(s) simOut.j2pf(s) simOut.j3pf(s) ...
                simOut.j4pf(s) simOut.j5pf(s) simOut.j6pf(s)] ...
                + [j0pi j1pi j2pi j3pi j4pi j5pi j6pi];
            count = count + 1
            i = i + 1;
        catch
            "error"
            pause(0.25)
        end
    end
    %trajTotal = cat(3, trajTotal, traj);
    trajTotal(:,:,((m-1)*trajPerChunk+1):((m)*trajPerChunk)) = [traj trajAngs];
    %jointPosTotal = [jointPosTotal; jointPos];
    jointPosTotal(((m-1)*trajPerChunk+1):((m)*trajPerChunk),:) = jointPos;

    csvwrite('traj_with_angs_10k.txt', trajTotal)
    csvwrite('jointPos_with_angs_10k.txt',jointPosTotal)
    m = m+1;
end




toc

%tes = load('data.mat', '-ASCII');