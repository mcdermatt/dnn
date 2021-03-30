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

%Work flow for case of constant human
%1) set random initial joint angles
%2) get cartesian position of hand
%3) perturb hand of human with unit force in 6 directions, measure 
%   displacement to obtain virtual impedance 
%4) Save data on joint angles and endpoint impedance

