%Generates trajectories with random sinusoidal variation

%save partial snapshots of each simulation as different trajectories
%   This should provide valuble about trajs with nonzero initial velocities

%TODO: optimize trajPts -> should I have more???
%I tried lowering timestep by a factor of 10 to 0.001 YOLO?
%   messing with try/ catch

beep off
numTraj = 1;
trajPerChunk = 1;
trajPts = 128; %number of points in each trajectory
trajTotal = zeros(trajPts,3,numTraj);
jointPosTotal = zeros(numTraj,7);

tic
count = 0;

m = 1;
while m <= (floor(numTraj/trajPerChunk))

    
    traj = zeros(trajPts, 3, trajPerChunk);
    jointPos = zeros(trajPerChunk, 7);    

    %run a few times
    %for i = 1:trajPerChunk
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

            j0vi = 10*randn();
            j1vi = 10*randn();
            j2vi = 10*randn();
            j3vi = 10*randn();
            j4vi = 10*randn();
            j5vi = 10*randn();
            j6vi = 10*randn();

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

            %create random time varying forces
            A = 3*randn(3,1); %amplitude
            B = 10*randn(3,1); %frequency
            C = randn(3,1); %phase

            timevec = ((0:1000)/500)';
            fz = timeseries(A(1)*sin(B(2)*timevec + C(1)),timevec);
            fx = timeseries(A(2)*cos(B(2)*timevec + C(2)),timevec);
            fy = timeseries(A(3)*sin(B(3)*timevec + C(3)),timevec);
            simOut = sim('human7DOF.slx');

            %TODO -> only look at back half of data so we are starting at a
            %nonzero velocity
            x = simOut.x([(floor(length(simOut.x)/2)):(length(simOut.x))]);
            y = simOut.y([(floor(length(simOut.y)/2)):(length(simOut.y))]);
            z = simOut.z([(floor(length(simOut.z)/2)):(length(simOut.z))]);

            startPos = [x(1) y(1) z(1)];

            %get array of xyz points in trajectory
            for j = 1:trajPts
                s = j*floor(length(x)/trajPts);
                traj(j,:,i) = [x(s) y(s) z(s)] - startPos;   
            end

            %get joint angles at final point
            jointPos(i,:) = [simOut.j0pf(s) simOut.j1pf(s) simOut.j2pf(s) simOut.j3pf(s) ...
                simOut.j4pf(s) simOut.j5pf(s) simOut.j6pf(s)] ...
                + [j0pi j1pi j2pi j3pi j4pi j5pi j6pi];
            count = count + 1
            i = i + 1;
        catch
            "error" %#ok<NOPTS>
        end
    end

    %trajTotal = cat(3, trajTotal, traj);
    trajTotal(:,:,((m-1)*trajPerChunk+1):((m)*trajPerChunk)) = traj;
    %jointPosTotal = [jointPosTotal; jointPos];
    jointPosTotal(((m-1)*trajPerChunk+1):((m)*trajPerChunk),:) = jointPos;

    csvwrite('traj_random.txt', trajTotal)
    csvwrite('jointPos_random.txt',jointPosTotal)
    
    m = m+1;
end




toc