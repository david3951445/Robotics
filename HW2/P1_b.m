clc; clear;

%% kinemic table
syms a1 a2 a3 a4 d1 d2 d3 d4
t1 = [d1    0   90  0
      0     a2  0   0
      0     0   -90 0
      d4	0   90  0
      0     0   -90 0
      0     0   0   0];
 
syms a1 a3
t2 = [0     a1	-90	0
      0     a2	0   0
      0     a3	0   0
      0 	0	-90	0
      0     0   90  0
      0     0   0   0];

% PUMA 560
syms a1 a3
t3 = [0     0	-90	0
      0     a2	0   0
      d3    a3	90  0
      d4 	0	-90	0
      0     0   90  0
      0     0   0   0];

%
t4 = [d1     0	0	0
      0     d2	0   0
      0    d3	0  0];
  
table = t4;
max = size(table, 1);
%% A matrix
syms nx ny nz ox oy oz ax ay az px py pz
noap = [nx ox ax px; ny oy ay py; nz oz az pz; 0 0 0 1];

% A
for i = 1 : max
    [A(:, :, i), A_inv(:, :, i)] = getA(i, table(i, :));
end
% A_inv
% D4 = [eye(3) [0; 0; d4]; 0 0 0 1]

% inverse of A
Inv = eye(4);
for i = 1 : max
%     A_inv(:, :, i)
    Inv = A_inv(:, :, i)*Inv;
end
Inv

% T
T = eye(4);
for i = 1 : max
    T = T*A(:, :, i);
end
T


function [A, A_inv] = getA(n, arr) % transformation matrix of joint n in D-H model
d = arr(1); a = arr(2); al = arr(3)/180*pi;
c = sym(['c' num2str(n)]); s = sym(['s' num2str(n)]);

A = [c    -s*cos(al)    s*sin(al)     a*c
     s    c*cos(al)     -c*sin(al)    a*s
     0    sin(al)       cos(al)       d
     0    0             0             1];
 
A_inv = subs(inv(A(:, :, 1)), c^2 + s^2, 1);
end