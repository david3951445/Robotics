function T = config2noap(config) % convert robot configuration (x, y, z, ϕ, θ, ψ) to noap matrix
ph = config(4)/180*pi;
th = config(5)/180*pi;
ps = config(6)/180*pi;

% attitude
R = [cos(ph)	-sin(ph)	0   0
     sin(ph)    cos(ph)     0   0
     0          0           1   0
     0          0           0   1];
 
P = [cos(th)	0	sin(th)	0
     0          1   0       0
     -sin(th)   0   cos(th) 0
     0          0	0       1];
 
Y = [cos(ps)	-sin(ps)    0   0
    sin(ps)     cos(ps)     0   0
    0           0           1   0
    0           0           0   1];
T = R*P*Y; 

% position
T(1:3, 4) = config(1:3);

end

