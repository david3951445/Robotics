classdef robot
    % parameter of RV-M2 robot
    
    properties (Constant)
        % kinematic table
        %        d a    alpha   theta
        table = [0 120  -90     0;
                 0 250  0       0;
                 0 260  0       0;
                 0 0    -90     0;
                 0 0    90      0;
                 0 0    0       0;]
             
        % angle range
        range = [-150   150;
                 -30    100;
                 -120   0;
                 -110   110;
                 -180   180;
                 -180   180;]
    end
    
    properties
        num % number of axis
        d, a, al, th % same as the values in table but change unit
    end
    
    methods
        function obj = robot()
            obj.num = length(obj.range);
            obj.d   = obj.table(:, 1)/1000; % m
            obj.a   = obj.table(:, 2)/1000; % m
            obj.al  = obj.table(:, 3)/180*pi; % rad
        end
        
        function y = A(obj, n, th) % transformation matrix of joint n in D-H model
            d  = obj.d(n);
            a  = obj.a(n);
            al = obj.al(n);
            
            y = [cos(th)    -sin(th)*cos(al)    sin(th)*sin(al)     a*cos(th);
                 sin(th)    cos(th)*cos(al)     -cos(th)*sin(al)    a*sin(th);
                 0          sin(al)             cos(al)             d
                 0          0                   0                   1]; 
        end
        
        % Determine whether the angle exceeds the working range. Return
        % those joints that are out of range.
        function y = isOutofRange(obj, theta) 
            ind = ones(obj.num, 1);
            for i = 1 : obj.num
                if obj.range(i, 1) <= theta(i) && theta(i) <= obj.range(i, 2)
                    ind(i) = 0;
                end
            end
            y = find(ind);
        end
    end
end
%#ok<*PROPLC>
