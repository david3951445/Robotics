clc; clear; close all
addpath(genpath('function')) % use functions in the "function" forder
addpath(genpath('class'))

%% parameters
dt = 0.002; t_acc = 0.202; T = 0.5;
P1 = [0 0 -1 .4; -1 0 0 -.3; 0 1 0 .1; 0 0 0 1]; % POS1
P2 = [1 0 0 .3; 0 -1 0 .3; 0 0 -1 .2; 0 0 0 1]; % POS2
P3 = [0 1 0 .4; 0 0 -1 .2; -1 0 0 -.3; 0 0 0 1]; % POS3

%% path generation
rb = RVM2(); % the robot is RV-M2

traj_C = PATH({P1, P2, P3}, [0, T, 2*T], [0, t_acc, 0], rb, 'Cartesian'); % Cartesian move
traj_J = PATH({P1, P2, P3}, [0, T, 2*T], [0, t_acc, 0], rb, 'Joint'); % Joint move

%% print
Print(traj_C, 'Cartesian')
Print(traj_J, 'Joint')

rmpath(genpath('function'))
rmpath(genpath('class'))