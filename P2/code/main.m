clc; clear; close all

%% add path (use files in forder)
addpath(genpath('function')) 
addpath(genpath('class'))

%% parameters
dt = 0.002; t_acc = 0.202; T = 0.5;
P1 = [0 0 -1 .4; -1 0 0 -.3; 0 1 0 .1; 0 0 0 1]; % POS1
P2 = [1 0 0 .3; 0 -1 0 .3; 0 0 -1 .2; 0 0 0 1]; % POS2
P3 = [0 1 0 .4; 0 0 -1 .2; -1 0 0 -.3; 0 0 0 1]; % POS3
% P4 = [0 0 -1 .6; -1 0 0 -.1; 0 1 0 .3; 0 0 0 1] % POS4

%% path generation
rb = RVM2(); % the robot is RV-M2

desiredPoints = {P1, P2, P3};
desiredPointsTime = [0, T, 2*T];
transitionSize = [0, t_acc, 0];

traj_C = PATH(desiredPoints, desiredPointsTime, transitionSize, rb, 'Cartesian'); % Cartesian move
traj_J = PATH(desiredPoints, desiredPointsTime, transitionSize, rb, 'Joint'); % Joint move

%% output results 
Print(traj_C, 'Cartesian')
Print(traj_J, 'Joint')

%% remove path
rmpath(genpath('function')) 
rmpath(genpath('class'))