clc; clear;
rb = RVM2; % the robot is RV-M2

%% forward
% input
theta = [90 0 -90 45 -45 0];

% output
T6 = forward(rb, theta); % get noap
Print1(T6)

%% inverse
% input
switch 3 % choose of input format
    case 1 % (n, o, a, p)
        c = 1/sqrt(2);
        T = [-c     0       -c      0
             0.5    c       -0.5    0.37
             0.5    -c      -0.5    0.26
             0      0       0       1];
         
    case 2 % (x, y, z, ϕ, θ, ψ)
        configuration = [0 0.325 -0.158 171.318 85.019 -119.622];
        T = config2noap(configuration);
        
    case 3 % result of foward
        T = T6;
end

% output
joint = inverse(rb, T); % get joint angles
Print2(rb, joint)