% robot configuration (x, y, z, ϕ, θ, ψ) -> noap matrix
% configuration unit : rad
function T = config2noap(config)
% toolbox
T = [eul2rotm(config(4:6)', 'ZYZ') config(1:3); 0 0 0 1];

% my method
% ph = config(4);
% th = config(5);
% ps = config(6);
% 
% % attitude
% R = [cos(ph)	-sin(ph)	0   0
%      sin(ph)    cos(ph)     0   0
%      0          0           1   0
%      0          0           0   1];
%  
% P = [cos(th)	0	sin(th)	0
%      0          1   0       0
%      -sin(th)   0   cos(th) 0
%      0          0	0       1];
%  
% Y = [cos(ps)	-sin(ps)    0   0
%     sin(ps)     cos(ps)     0   0
%     0           0           1   0
%     0           0           0   1];
% T = R*P*Y; 
% 
% % position
% T(1:3, 4) = config(1:3);
end

