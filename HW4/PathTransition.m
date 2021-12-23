% from ROBOT MANIPULATOR book, p.136, Fig 5.13
clc; clear;
dt = 0.01;
acc = 0.2; % 值越大，頂點處越平滑
A = 0; B = 1; C = 2;
dB = A - B; dC = C - B;
T = 1; t_acc = T*acc;
t = -t_acc : dt : t_acc;
h = (t + t_acc)/(2*t_acc);
q = zeros(1, length(t)); % Poly_AB'
for i = 1 : length(t)
    q(i) = ((dC*t_acc/T + dB)*(2 - h(i))*h(i)^2 - 2*dB)*h(i) + B + dB;
end

% L_AB
t1 = -t_acc : dt : 0;
L_AB = -dB*(t1 + t_acc)/(0 + t_acc) + A;
% L_BC
t2 = 0 : dt : T;
L_BC = dC*t2/T + B;
plot(t1, L_AB, t2, L_BC, t, q)