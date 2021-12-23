% Polynomial Path Planning
clc; clear;
% I.C
tf = 1; % final time
x(1).p = 0; x(1).v = 0; % start position & velocity
x(2).p = 10; x(2).v = 10; % end position & velocity
% coefficients of 3rd-order polynomial
a = [x(1).p x(1).v 0 0];
a(3) = 3*(x(2).p - x(1).p)/tf^2 - 2*x(1).v/tf - x(2).v/tf;
a(4) = -2*(x(2).p - x(1).p)/tf^3 + (x(2).v + x(1).v)/tf^2;
% plot
t = 0 : 0.01: tf;
poly = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3;
plot(t, poly)