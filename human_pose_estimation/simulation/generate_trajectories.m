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
numTraj = 5;
trajPerChunk = 1;
ptsPerSec = 10; %polling of end effector state this many times per sec
trajLength = 1; %number of secs to run traj (should match value in simulink)
trajPts = trajLength*ptsPerSec; %number of points in each trajectory
trajTotal = zeros(trajPts,6,numTraj);
jointPosTotal = zeros(numTraj,9);
angsRelativeToStart = true; %false means that angles of ball relative to world frame
                             %true means angles of ball are relative to starting position

%NOTE: This works WAAAYY faster withtout visual simulation on 

tic
count = 0;

m = 1;
while m <= (floor(numTraj/trajPerChunk))

        traj = zeros(trajPts, 3, trajPerChunk);
        trajAngs = zeros(trajPts, 3, trajPerChunk);
        jointPos = zeros(trajPerChunk, 9);    

        %run a few t imes
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

            % case of random time varying forces
%             A = 6*randn(3,1); %amplitude
%             B = 10*randn(3,1); %frequency
%             C = randn(3,1); %phase
%             timeLen = 5;
%             timevec = ((0:1000)/timeLen)';
%             fz = timeseries(A(1)*sin(B(2)*timevec + C(1)),timevec);
%             fx = timeseries(A(2)*cos(B(2)*timevec + C(2)),timevec);
%             fy = timeseries(A(3)*sin(B(3)*timevec + C(3)),timevec);
%             
%             %case of constant cartesian external forces (no gravity)
            mult = 1;
            fx = [0 mult*randn()];
            fy = [0 mult*randn()];
            fz = [0 mult*randn()];

            %case of reversing cartesian external forces ---------------
            %   Want initial jolt and then negative jolt of equal magnitude
            %   halfway through
%             timeLen = 1;
%             timevec = ((0:0.1:100)/timeLen)';
%             %init time series structures
%             fx = timeseries(0,timevec);
%             fy = timeseries(0,timevec);
%             fz = timeseries(0,timevec); 
% 
%             %loop through to make random point to point movements
%             for count = 1:10
%                 mult = 3;
%                 rx = mult*randn();
%                 ry = mult*randn();
%                 rz = mult*randn();
%                 fx(1).Data(20*count-9) = rx;
%                 fy(1).Data(20*count-9) = ry;
%                 fz(1).Data(20*count-9) = rz;
% 
%                 fx(1).Data(20*count) = -rx;
%                 fy(1).Data(20*count) = -ry;
%                 fz(1).Data(20*count) = -rz;
%             end
%           --------------------------------------------------------------           
            model = 'human9DOF';
            simOut = sim(model);
    
            %get array of xyz points in trajectory
            for j = 1:trajPts
                s = j*floor(length(simOut.x)/trajPts);
                traj(j,:,i) = [simOut.x(s) simOut.y(s) simOut.z(s)];
                trajAngs(j,:,i) = [simOut.ang(s,1) simOut.ang(s,2) simOut.ang(s,3)];
            end
            
            startPos = traj(1,:,i);
            
            if angsRelativeToStart == true
                startAngs =trajAngs(1,:,i);
            else
                startAngs =[0 0 0];
            end
            
            traj = traj - startPos;
            trajAngs = trajAngs - startAngs;

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
% 
%     csvwrite('data/traj_9DOF_long.txt', trajTotal)
%     csvwrite('data/jointPos_9DOF_long.txt',jointPosTotal) %only useful
% %     for training the network, needed for utils rotation func (should fix
% %     later for efficiency)
%     csvwrite('data/jointPath_long.txt', jointPath);
    
%     csvwrite('data/traj_9DOF_1.txt', trajTotal)
%     csvwrite('data/jointPos_9DOF_1.txt',jointPosTotal)
%     csvwrite('data/jointPath.txt', jointPath);
 
%     csvwrite('data/traj_9DOF_rel2start100k.txt', trajTotal)
%     csvwrite('data/jointPos_9DOF_rel2start100k.txt',jointPosTotal)


    m = m+1;
end




toc
