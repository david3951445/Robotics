clc; clear; close all
addpath(genpath('functions'))
rb = RVM2();
%% parameters
dt = 0.002; t_acc = 0.2; T = 0.5;
P1 = [0 0 -1 40; -1 0 0 -30; 0 1 0 10; 0 0 0 1]; % POS1
P2 = [1 0 0 30; 0 -1 0 30; 0 0 -1 20; 0 0 0 1]; % POS2
P3 = [0 1 0 40; 0 0 -1 20; -1 0 0 -30; 0 0 0 1]; % POS3
% example in Richard's robot manipulator book
% dt = 0.01; t_acc = 0.1; T = 10*t_acc;
% P1 = [-1 0 0 0; 0 1 0 10; 0 0 -1 10; 0 0 0 1];
% P2 = [0 0 1 0; 1 0 0 0; 0 1 0 10; 0 0 0 1];
% P3 = [-1 0 0 10; 0 -1 0 0; 0 0 1 10; 0 0 0 1];

%% find ABC trajectory
cf_D = get_Dconfig(P1, P2);
r = (T - t_acc)/T;
A = P1*D(r, cf_D); B = P2; C = P3;
dA = get_Dconfig(B, A); % find A - B
dC = get_Dconfig(B, C); % find C - B

% minimize the change of orientation
if abs(dC(6) - dA(6)) > pi/2
    dA(6) = dA(6) + pi;
    if abs(dC(6) - dA(6)) > pi % ex : 10 and 350
        dA(6) = dA(6) - 2*pi;
    end
    
    dA(5) = -dA(5);
end

% trajectory (set B as origin)
t = -t_acc : dt : T - dt;
arr = zeros(6, length(t));
traj = struct('Dconfig',{arr, arr, arr}); % {pos, vel, acc}
cf_A = noap2config(A);
cf_B = noap2config(B);
cf_C = noap2config(C);
for k = 1 : length(t) % -tacc ~ T
    if t(k) < t_acc % transition portion, -tacc ~ tacc
        h = (t(k) + t_acc)/(2*t_acc);
        CTB = dC(1:5)*t_acc/T + dA(1:5);
        traj(1).Dconfig(1:5, k) = (CTB*(2 - h)*h^2 - 2*dA(1:5))*h + dA(1:5) ; % position
        traj(2).Dconfig(1:5, k) = (CTB*(1.5 - h)*2*h^2 - dA(1:5))/t_acc; % velocity
        traj(3).Dconfig(1:5, k) = CTB*(1 - h)*3*h/t_acc^2; % acceleration
    else % linear portion, tacc ~ T
        h = t(k)/T;
        
        traj(1).Dconfig(6, k) = dC(6);
        traj(1).Dconfig(1:5, k) = dC(1:5)*h; % position
%         B*D(1, traj(1).Dconfig(:, k));
        
        traj(2).Dconfig(1:5, k) = dC(1:5)/T; % velocity
        traj(3).Dconfig(1:5, k) = 0; % acceleration
    end
end
% shift origin to B
traj(1).Dconfig(1:5, :) = traj(1).Dconfig(1:5, :) + cf_B(1:5);

%% find P1~A trajectory
dP = cf_D;
T1 = T - t_acc;

t1 = 0 : dt : T1 - dt;
arr = zeros(6, length(t1));
traj1 = struct('Dconfig',{arr, arr, arr}, 't', t1); % {pos, vel, acc}
cf_P1 = noap2config(P1);

for k = 1 : length(t1) % -T ~ -tacc
    h = t1(k)/T1;
    
%     traj1(1).Dconfig(6, k) = cf_A(6);
%     traj1(1).Dconfig(1:5, k) = dP(1:5)*h; % pos
%     noap = P1*D(1, traj1(1).Dconfig(:, k));
    noap = P1*D(t1(k)/T, cf_D);
    traj1(1).Dconfig(:, k) = noap2config(noap);
    
    traj1(2).Dconfig(1:5, k) = dP(1:5)/T1; % vel
    traj1(3).Dconfig(1:5, k) = 0; % acc
end

% shift origin to B
% traj(1).Dconfig(1:5, :) = traj(1).Dconfig(1:5, :) + cf_P1(1:5);

% combine two portion (-T ~ tacc & -tacc ~ T)
t = 0 : dt : 2*T - dt;
arr = zeros(6, length(t));
traj2 = struct('Dconfig',{arr, arr, arr}, 't', t); % {pos, vel, acc}
for i = 1 : 3
    traj2(i).Dconfig = cat(2, traj1(i).Dconfig, traj(i).Dconfig);
end

%% print
for i = 1 : 1 % pos, vel ,acc
    Print(traj1(i).Dconfig, traj1(i).t, i)
end

%% debug, print config. unit : cm, deg
[dC(1:3); dC(4:6)*180/pi]
B*D(1, [dC(1:5)*0.1; dC(6)])
[cf_B(1:3); cf_B(4:6)*180/pi]
[cf_C(1:3); cf_C(4:6)*180/pi]
[cf_C(1:3); cf_C(4:6)*180/pi] - [cf_B(1:3); cf_B(4:6)*180/pi]


%% functions
% get x, y, z, psi, theta, phi of D. start potin : P1, end point : P2
function config = get_Dconfig(P1, P2)
n1 = P1(1:3, 1); o1 = P1(1:3, 2); a1 = P1(1:3, 3); p1 = P1(1:3, 4);
n2 = P2(1:3, 1); o2 = P2(1:3, 2); a2 = P2(1:3, 3); p2 = P2(1:3, 4);

% note : no atan2's judgement currently
x = n1'*(p2 - p1);
y = o1'*(p2 - p1);
z = a1'*(p2 - p1);
psi = atan2(o1'*a2, n1'*a2);
theta = atan2(sqrt((n1'*a2)^2 + (o1'*a2)^2), a1'*a2);
% find phi
V = 1 - cos(theta); CVC = cos(psi)^2*V + cos(theta);
s = -sin(psi)*cos(psi)*V*(n1'*n2) + CVC*(o1'*n2) - sin(psi)*sin(theta)*(a1'*n2);
c = -sin(psi)*cos(psi)*V*(n1'*o2) + CVC*(o1'*o2) - sin(psi)*sin(theta)*(a1'*o2);
phi = atan2(s, c);

config = [x; y; z; phi; theta; psi];
end

function output = D(r, config) % D matrix, P1*D = P2
x = config(1);
y = config(2);
z = config(3);
phi = config(4);
theta = config(5);
psi = config(6);

% find Tr
Tr = eye(4); Tr(1:3, 4) = [r*x; r*y; r*z];
% find Ra
Ra = eye(4);
sp = sin(psi); cp = cos(psi);
st = sin(r*theta); ct = cos(r*theta); V = 1 - ct;
Ra(1:3, 1:3) = [sp^2*V + ct, -sp*cp*V, cp*st;...
    -sp*cp*V, cp^2*V + ct, sp*st; -cp*st, -sp*st, ct];
% find Ro
Ro = eye(4); Ro(1:2, 1:2) = [cos(r*phi) -sin(r*phi); sin(r*phi) cos(r*phi)];

output = Tr*Ra*Ro;
end