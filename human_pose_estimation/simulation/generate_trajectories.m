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
numTraj = 1;
trajPerChunk = 1;
trajPts = 10; %number of points in each trajectory
trajTotal = zeros(trajPts,6,numTraj);
jointPosTotal = zeros(numTraj,9);

%NOTE: This works WAAAYY faster withtout visual simulation on 

tic
count = 0;

m = 1;
while m <= (floor(numTraj/trajPerChunk))

        traj = zeros(trajPts, 3, trajPerChunk);
        trajAngs = zeros(trajPts, 3, trajPerChunk);
        jointPos = zeros(trajPerChunk, 9);    

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
            j7pi = rand()*90 - 45;
            j8pi = rand()*100 - 50;

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
            j7ll = -j7pi - 90;
            j7ul = -j7pi + 90;
            j8ll = -j8pi - 55;
            j8ul = -j8pi + 55;
            
            j0vi = 0;
            j1vi = 0;
            j2vi = 0;
            j3vi = 0;
            j4vi = 0;
            j5vi = 0;
            j6vi = 0;
            j7vi = 0;
            j8vi = 0;

            %case of constant cartesian external forces (no gravity)
            mult = 1;
            fx = [0 mult*randn()];
            fy = [0 mult*randn()];
            fz = [0 mult*randn()];
            model = 'human9DOF.slx';
            simOut = sim(model);
            
            %using parallel simulation & simulation manager
%             load_system(model);
%             numSims = 4;
%             in(1:numSims) = Simulink.SimulationInput(model);
%             for i = 1:numSims;
%                 in(i) = setBlockParameter(in(i), [model '/
%             simOut = parsim(in);
    
            startPos = [simOut.x(1) simOut.y(1) simOut.z(1)];

            %get array of xyz points in trajectory
            for j = 1:trajPts
                s = j*floor(length(simOut.x)/trajPts);
                traj(j,:,i) = [simOut.x(s) simOut.y(s) simOut.z(s)] - startPos;
                trajAngs(j,:,i) = [simOut.ang(s,1) simOut.ang(s,2) simOut.ang(s,3)];
            end

            %get joint angles at final point
            jointPos(i,:) = [simOut.j0pf(s) simOut.j1pf(s) simOut.j2pf(s) simOut.j3pf(s) ...
                simOut.j4pf(s) simOut.j5pf(s) simOut.j6pf(s) simOut.j7pf(s) simOut.j8pf(s)] * 180 / pi ...
                + [j0pi j1pi j2pi j3pi j4pi j5pi j6pi j7pi j8pi];
            count = count + 1
            i = i + 1;
        
            %array for storing position of joints at each timestep
            jointPath = (180 / pi) *[simOut.j0pf simOut.j1pf simOut.j2pf simOut.j3pf ...
                simOut.j4pf simOut.j5pf simOut.j6pf simOut.j7pf simOut.j8pf] ...
                + [j0pi j1pi j2pi j3pi j4pi j5pi j6pi j7pi j8pi];
            %   10k/60 = 167
            jointPath = jointPath(1:167:end,:);
            
        catch
            "error"
            pause(0.25)
        end
    end
    %trajTotal = cat(3, trajTotal, traj);
    trajTotal(:,:,((m-1)*trajPerChunk+1):((m)*trajPerChunk)) = [traj trajAngs];
    %jointPosTotal = [jointPosTotal; jointPos];
    jointPosTotal(((m-1)*trajPerChunk+1):((m)*trajPerChunk),:) = jointPos;

%     csvwrite('data/traj_9DOF_1.txt', trajTotal)
%     csvwrite('data/jointPos_9DOF_1.txt',jointPosTotal)
%     csvwrite('data/jointPath.txt', jointPath);

    m = m+1;
end




toc

%tes = load('data.mat', '-ASCII');