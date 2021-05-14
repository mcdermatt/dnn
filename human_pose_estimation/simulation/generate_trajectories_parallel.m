%TODO
%  x -> going to run into memory issues if we try to save every detail from
%      every sim
%       need to run 1000 or so sims and take the useful data from them then
%       clear the outputs
%  x -> Need to make sure data coming from parsim is accurate and useful
%  x -> error handling
%       what to do when I skip a trial with an error and have a traj of all
%       zeros?
%  _-> Need to train on trajectories of NONZERO STARTING VELOCITY 


%REMEMBER TO SET SIMULINK MODEL TO 2s

beep off
tic

numTraj = 100000;
% trajPerChunk = 1; depricated, now numsims
trajPts = 10; %number of points in each trajectory
trajTotal = zeros(trajPts,6,numTraj);
% trajTotal = [];
jointPosTotal = zeros(numTraj,9);
% jointPosTotal = [];
numSims = 1000;% Max number of simulations to be stored in RAM at a time
% can run 1000 sims in 146s (with 12 workers)
angsRelativeToStart = false; %false means that angles of ball relative to world frame
                             %true means angles of ball are relative to starting position


m = 1;

while m <= (floor(numTraj/numSims))
    in(numSims) = Simulink.SimulationInput;
    for idx = 1:numSims
        % Need to populate the model name since we get any empty array by default
        in(idx).ModelName = 'human9DOF';

        %get random joint angles within limits
        j0pi = rand()*20-10;
        in(idx) = in(idx).setVariable('j0pi', j0pi);
        j1pi = rand()*20 - 10;
        in(idx) = in(idx).setVariable('j1pi', j1pi);
        j2pi = rand()*10 - 5;
        in(idx) = in(idx).setVariable('j2pi', j2pi);
        j3pi = rand()*110 - 90;
        in(idx) = in(idx).setVariable('j3pi', j3pi);
        j4pi = rand()*120 - 30;
        in(idx) = in(idx).setVariable('j4pi', j4pi);
        j5pi = rand()*120 -30;
        in(idx) = in(idx).setVariable('j5pi', j5pi);
        j6pi = -rand()*130;
        in(idx) = in(idx).setVariable('j6pi', j6pi);
        j7pi = rand()*90 - 45;
        in(idx) = in(idx).setVariable('j7pi', j7pi);
        j8pi = rand()*100 - 50;
        in(idx) = in(idx).setVariable('j8pi', j8pi);

        %update joint limits
        j0ll = -j0pi - 25;
        in(idx) = in(idx).setVariable('j0ll', j0ll);
        j0ul = -j0pi + 25;
        in(idx) = in(idx).setVariable('j0ul', j0ul);
        j1ll = -j1pi - 30;
        in(idx) = in(idx).setVariable('j1ll', j1ll);
        j1ul = -j1pi + 30;
        in(idx) = in(idx).setVariable('j1ul', j1ul);
        j2ll = -j2pi - 7.5;
        in(idx) = in(idx).setVariable('j2ll', j2ll);
        j2ul = -j2pi + 60;
        in(idx) = in(idx).setVariable('j2ul', j2ul);
        j3ll = -j3pi - 90; 
        in(idx) = in(idx).setVariable('j3ll', j3ll);
        j3ul = -j3pi + 20;
        in(idx) = in(idx).setVariable('j3ul', j3ul);
        j4ll = -j4pi - 30;
        in(idx) = in(idx).setVariable('j4ll', j4ll);
        j4ul = -j4pi + 90;
        in(idx) = in(idx).setVariable('j4ul', j4ul);
        j5ll = -j5pi - 180;
        in(idx) = in(idx).setVariable('j5ll', j5ll);
        j5ul = -j5pi + 180;
        in(idx) = in(idx).setVariable('j5ul', j5ul);
        j6ll = -j6pi - 130;
        in(idx) = in(idx).setVariable('j6ll', j6ll);
        j6ul = -j6pi;
        in(idx) = in(idx).setVariable('j6ul', j6ul);
        j7ll = -j7pi - 90;
        in(idx) = in(idx).setVariable('j7ll', j7ll);
        j7ul = -j7pi + 90;
        in(idx) = in(idx).setVariable('j7ul', j7ul);
        j8ll = -j8pi - 55;
        in(idx) = in(idx).setVariable('j8ll', j8ll);
        j8ul = -j8pi + 55;
        in(idx) = in(idx).setVariable('j8ul', j8ul);

        j0vi = 0;
        in(idx) = in(idx).setVariable('j0vi', j0vi);
        j1vi = 0;
        in(idx) = in(idx).setVariable('j1vi', j1vi);
        j2vi = 0;
        in(idx) = in(idx).setVariable('j2vi', j2vi);
        j3vi = 0;
        in(idx) = in(idx).setVariable('j3vi', j3vi);
        j4vi = 0;
        in(idx) = in(idx).setVariable('j4vi', j4vi);
        j5vi = 0;
        in(idx) = in(idx).setVariable('j5vi', j5vi);
        j6vi = 0;
        in(idx) = in(idx).setVariable('j6vi', j6vi);
        j7vi = 0;
        in(idx) = in(idx).setVariable('j7vi', j7vi);
        j8vi = 0;
        in(idx) = in(idx).setVariable('j8vi', j8vi);

%         % case of random time varying forces
        A = 3*randn(3,1);     %amplitude
        B = 10*randn(3,1);    %frequency
        C = randn(3,1);       %phase
        timevec = ((0:1000)/5)';
        fz = timeseries(A(1)*sin(B(2)*timevec + C(1)),timevec);
        fx = timeseries(A(2)*cos(B(2)*timevec + C(2)),timevec);
        fy = timeseries(A(3)*sin(B(3)*timevec + C(3)),timevec);

        %case of constant cartesian external forces (no gravity)
%         mult = 1;
%         fx = [0 mult*randn()];
        in(idx) = in(idx).setVariable('fx', fx);
%         fy = [0 mult*randn()];
        in(idx) = in(idx).setVariable('fy', fy);
%         fz = [0 mult*randn()];
        in(idx) = in(idx).setVariable('fz', fz);   
    end

    simOutPar = parsim(in);

    traj = zeros(trajPts, 3, numSims); 
    trajAngs = zeros(trajPts, 3, numSims);
    jointPos = zeros(numSims, 9);
    
    for i = 1:numSims %for each sim in memory

      %if there were no errors on current sim
      if size(simOutPar(i).ErrorMessage,1) == 0

      % update traj (shape of traj is [10, 6, #trajs]) --------------
      % get array of xyz pts in trajectory
        for j = 1:trajPts
            s = j*floor(length(simOutPar(i).x)/trajPts); %looking at all points
%             s = j*floor(0.5*length(simOutPar(i).x)/trajPts) + floor(0.5*length(simOutPar(i).x)); %only looking at back half
            
            %default: looking at absolute positions (oculus)
            %   SET MEASUREMENT TO WORLD FRAME
            traj(j,:,i) = [simOutPar(i).x(s) simOutPar(i).y(s) simOutPar(i).z(s)];
            trajAngs(j,:,i) = [simOutPar(i).ang(s,1) simOutPar(i).ang(s,2) simOutPar(i).ang(s,3)];

            
            %iPhone: 6 axis gyro/ acclerometer
            %   Set measurement to follower(?)
%             TODO: Figure out what to do about rotation with accelerometer
%             data
%             traj(j,:,i) = [simOutPar(i).ax(s) simOutPar(i).ay(s) simOutPar(i).az(s)];
%             trajAngs(j,:,i) = [simOutPar(i).wx(s) simOutPar(i).wy(s) simOutPar(i).wz(s)];
            
        end
        
        startPos = traj(1,:,i);
            
        if angsRelativeToStart == true %set false for iphone
            startAngs =trajAngs(1,:,i);
        else
            startAngs =[0 0 0];
        end

        traj(:,:,i) = traj(:,:,i) - startPos;
        trajAngs(:,:,i) = trajAngs(:,:,i) - startAngs;

        %update jointPos --------------------------------------------
        jointPos(i,:) = [simOutPar(i).j0pf(s) simOutPar(i).j1pf(s) simOutPar(i).j2pf(s) simOutPar(i).j3pf(s) ...
            simOutPar(i).j4pf(s) simOutPar(i).j5pf(s) simOutPar(i).j6pf(s) simOutPar(i).j7pf(s) simOutPar(i).j8pf(s)] * 180 / pi ...
            + [in(i).Variables(1).Value, in(i).Variables(2).Value, in(i).Variables(3).Value, ...
                in(i).Variables(4).Value, in(i).Variables(5).Value, in(i).Variables(6).Value, ...
                in(i).Variables(7).Value, in(i).Variables(8).Value, in(i).Variables(9).Value];
            %+ [j0pi j1pi j2pi j3pi j4pi j5pi j6pi j7pi j8pi];
      end
    end
    trajTotal(:,:,((m-1)*numSims+1):((m)*numSims)) = [traj trajAngs];
    jointPosTotal(((m-1)*numSims+1):((m)*numSims),:) = jointPos;

    m = m+1;
    clearvars simOutPar

    csvwrite('data/traj_9DOF.txt', trajTotal)
    csvwrite('data/jointPos_9DOF.txt',jointPosTotal)
    
end

    


toc