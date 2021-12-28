classdef RVM2
    % parameter of RV-M2 robot
    properties (Constant)
        % kinematic table
        %        d a    alpha   theta
        table = [0 120  -90     0
                0 250  0       0
                0 260  0       0
                0 0    -90     0
                0 0    90      0
                0 0    0       0]
        
        % joint angles range
        range = [-150   150
                -30    100
                -120   0
                -110   110
                -180   180
                -180   180]
    end
    
    properties
        num % number of axis
        d, a, al, th % same as the values in table but change unit
    end
    
    methods
        function obj = RVM2()
            obj.num = size(obj.range, 1);
            obj.d   = obj.table(:, 1)/1000; % m
            obj.a   = obj.table(:, 2)/1000; % m
            obj.al  = obj.table(:, 3)/180*pi; % rad
        end
        
        function y = A(obj, n, th) % transformation matrix of joint n in D-H model
            d  = obj.d(n);
            a  = obj.a(n);
            al = obj.al(n);
            
            y = [cos(th)    -sin(th)*cos(al)    sin(th)*sin(al)     a*cos(th)
                sin(th)    cos(th)*cos(al)     -cos(th)*sin(al)    a*sin(th)
                0          sin(al)             cos(al)             d
                0          0                   0                   1];
        end
        
        % Determine whether the angle exceeds the working range. Return
        % those joints that are out of range. (unit : degree)
        function y = isOutofRange(obj, theta)          
            ind = ones(obj.num, 1);
            for i = 1 : obj.num
                if obj.range(i, 1) <= theta(i) && theta(i) <= obj.range(i, 2)
                    ind(i) = 0;
                end
            end
            
            y = find(ind);
        end
        
        % forward kinemic
        function T6 = forward(obj, theta)
            T6 = eye(4); % noap matrix
            for i = 1 : obj.num
                T6 = T6*obj.A(i, theta(i)); % multiply from A1 to A6
            end
        end
        
        % inverse kinemic (unit : rad)
        function joint = inverse(obj, T6)
            theta = zeros(obj.num, 8); % joints answer
            OofR = logical(false(1, 8)); % out of reach judgment, bool values
            
            px = T6(1, 4); py = T6(2, 4); pz = T6(3, 4);
            
            % theta 1
            if px == 0 && py == 0 % (if a1 + a2c2 + a3c23 == 0)
                theta(1, 1:2) = [-pi/2, pi/2]; % default value, in fact, theta 1 satisfied -150 <= theta 1 <= 150
            else
                theta(1, 1:2) = [atan2(py, px) atan2(py, px) + pi];
            end
            
            for i = 1 : 2
                index = (1:2) + 2*(i-1); % just for indexing
                
                % follow the steps in the handout
                f1 = px*cos(theta(1, i)) + py*sin(theta(1, i)) - obj.a(1);
                f2 = -pz;
                
                % theta 2
                % g1, g2 are algebraic substitutions
                g1 = f1/f2;
                g2 = (f1^2 + f2^2 + obj.a(2)^2 - obj.a(3)^2)/(2*obj.a(2)*f2);
                
                % c2 denotes cos(theta 2), s2 denotes sin(theta 2)
                c2 = (g1*g2 + sqrt(g1^2 - g2^2 + 1))/(1 + g1^2); % Quadratic formula
                s2 = g2 - g1*c2;
                c2_ = (g1*g2 - sqrt(g1^2 - g2^2 + 1))/(1 + g1^2);
                s2_ = g2 - g1*c2_;
                
                try
                    theta(2, index) = [atan2(s2, c2) atan2(s2_, c2_)];
                catch % there is no solution -> out of reach
                    OofR(index) = 1;
                end
                
                % theta 3
                c3 = (f1^2 + f2^2 - obj.a(2)^2 - obj.a(3)^2)/(2*obj.a(2)*obj.a(3));
                s3 = sqrt(1 - c3^2);
                
                try
                    theta(3, index) = [atan2(s3, c3) atan2(-s3, c3)];
                catch % there is no solution -> out of reach
                    % OofR(index) = 1;
                end
                
                % determine the arrangement of theta 2 and theta 3 from the equation "f1 = a2c2 + a3c23"
                % if (f1 - a2c2 - a3c23 != 0)
                if norm(f1 - obj.a(2)*cos(theta(2, index)) - obj.a(3)*cos(theta(2, index) + theta(3, index))) > 10^(-4)
                    theta(2, index) = theta(2, flip(index)); % rearrange
                end
            end
            
            theta(1, 1:4) = repelem(theta(1, 1:2), 2); % arrange theta 1
            
            for i = 1 : 4
                T3 = eye(4);
                for j = 1 : 3
                    T3 = T3*obj.A(j, theta(j, i));
                end
                T36 = T3\T6; % remove T3 (since it is known)
%                 T36 = T6;
%                 for j = 1 : 3
%                     T36 = obj.A(j, theta(j, i))\T36;
%                 end
                
                % theta 4 (if s5 != 0)
                ax = T36(1, 3); ay = T36(2, 3);
                if ax == 0 && ay == 0
                    theta(4, (1:2) + 2*(i-1)) = [-pi/2, pi/2]; % default value, in fact, -110 <= theta 4 <= 110
                else
                    theta(4, (1:2) + 2*(i-1)) = [atan2(ay, ax) atan2(ay, ax) + pi];
                end
            end
            
            theta(1:3, :) = repelem(theta(1:3, 1:4), 1, 2); % arrange theta 1~3
            OofR = repelem(OofR(1:4), 1, 2); % arrange OofR
            
            for i = 1 : 8
                T4 = eye(4);
                for j = 1 : 4
                    T4 = T4*obj.A(j, theta(j, i));
                end
                T46 = T4\T6; % remove T4
                
                % theta 5
                theta(5, i) = atan2(T46(1, 3), -T46(2, 3));
                
                % theta 6
                theta(6, i) = atan2(T46(3, 1), T46(3, 2));
            end
            
            joint.th = theta; joint.OofR = OofR; % return ans
        end
    end
end
%#ok<*PROPLC>
