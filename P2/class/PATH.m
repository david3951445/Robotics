classdef PATH
    %path planning (Cartesian move)
    % Input points' configuration, time, transition time to construct
    % correspond pos, vel, acc path.
    
    properties (Constant)
        dt = 0.002; % tume step
    end
    
    properties
        t % time array of path (0, dt, 2*dt, ...)
        tf % final time
        len_t % length of time
        num_p % number of point
        
        point % desired points in the configuration space
        path % desired path
        
        debug % for debug
    end
    
    methods
        function obj = PATH(noap, time, tacc, rb, method)
            %             addpath(genpath('functions'))
            
            %% t, tf, len_t, num_p
            obj.tf = time(length(time));
            obj.t = 0 : obj.dt : obj.tf;
            obj.len_t = length(obj.t);
            obj.num_p = length(noap);
            
            %% point
            for i = 1 : obj.num_p
                obj.point(i).noap = noap{i}; % noap matrix
                obj.point(i).config = noap2config(noap{i}); % x, y, z, phi, theta, psi
                obj.point(i).joint = noap2joint(noap{i}, rb); % θ1 ~ θ6
                obj.point(i).time = time(i); % time of this point
                obj.point(i).tacc = tacc(i); % transition time
                
                % Just for path_planning(), the designed variable in
                % polynomial path planning, i.e.
                % for Cartesianmove, point.val = point.config;
                % for Joint move, point.val = point.joint
                switch method
                    case 'Cartesian'
                        obj.point(i).val = obj.point(i).config;
                    case 'Joint'
                        obj.point(i).val = obj.point(i).joint;
                end
                
                % start & end index of transistion time of i-th point
                obj.point(i).ts_index(1) = round((time(i) - tacc(i))/obj.dt + 1); % start index
                obj.point(i).ts_index(2) = round((time(i) + tacc(i))/obj.dt + 1); % end index
            end
            
            %% path
            switch method
                case 'Cartesian'
                    %% configuration path
                    p = obj.path_planning();
                    for i = 1 : 3 % pos, vel, acc
                        obj.path(i).config = p(i).val;
                    end              
                    
                    %% joint path (obtain form "configuration path")
                    %-1 position (use inverse kinemic)
                    for i = 1 : obj.len_t
                        noap = config2noap(obj.path(1).config(:, i));
                        joint = noap2joint(noap, rb);
                        
                        % find the solution close to the previous moment
                        if i == 1 % the first moment does not exist the previous moment
                            joint_min = joint(:, 1);
                        else
                            for k = 1 : size(joint, 2)
                                d = norm(obj.path(1).joint(:, i-1) - joint(:, k));
                                
                                % k == 1 -> only one solution, no need to judgement
                                % k != 1 -> chose the closer one
                                if k == 1 || d < d_min
                                    joint_min = joint(:, k);
                                    d_min = d;
                                end
                            end
                        end
                        obj.path(1).joint(:, i) = joint_min; % the closest one
                        
                        % noap path
                        obj.path(1).noap(:, :, i) = noap;
                    end
                    
                    %-1-1 correct error (matrix singularity of inv(T3) in rb.inverse()) of joint 2 & 4 at t = 0.7 (temporary method)
%                     i = round(0.7/obj.dt + 1);
%                     obj.path(1).joint([2 4], i) = (obj.path(1).joint([2 4], i-1) + obj.path(1).joint([2 4], i+1))/2;
                    
                    %-1-2 correct "2π error" of joint 6 after t = 0.854 (temporary method)
                    i1 = round(0.854/obj.dt + 1);
                    for i = i1 : obj.len_t
                        obj.path(1).joint(6, i) = obj.path(1).joint(6, i) - 2*pi; % shift 2π to make path continuous
                    end
                    
                    %-2 velocity (numerical differentiation method)
                    obj.path(2).joint = numercial_diff(obj.path(1).joint, obj.dt);
                    
                    %-3 acceleration (numerical differentiation method)
                    obj.path(3).joint = numercial_diff(obj.path(2).joint, obj.dt);
                    
                case 'Joint'
                    %% joint path
                    p = obj.path_planning();
                    for i = 1 : 3 % pos, vel, acc
                        obj.path(i).joint = p(i).val;
                    end 
                    
                    %% configuration path (obtain from "joint path")
                    %-1 position (use forward kinemic)
                    for i = 1 : obj.len_t
                        noap = joint2noap(obj.path(1).joint(:, i), rb);
                        obj.path(1).config(:, i) = noap2config(noap);
                        
                        % noap path
                        obj.path(1).noap(:, :, i) = noap;
                    end         
                    
                    %-2 velocity (numerical differentiation method)
                    obj.path(2).config = numercial_diff(obj.path(1).config, obj.dt);
                    
                    %-3 acceleration (numerical differentiation method)
                    obj.path(3).config = numercial_diff(obj.path(2).config, obj.dt);
                    
                otherwise
                    disp('error, no this type\n')
            end
        end
    end
    
    methods (Access = private)
        % polynomial path planning
        function p = path_planning(obj)
            % initial
            p = struct;
            n = size(obj.point(1).val, 1);
            for i = 1 : 3 % pos, vel, acc
                p(i).val = zeros(n, obj.len_t);
            end
            
            %-1 linear portions
            for i = 1 : obj.num_p - 1 % N points -> N-1 line segment
                % start point (i-th point)
                j1 = obj.point(i).ts_index(2); % index
                c1 = obj.point(i).val;
                t1 = obj.point(i).time;

                % end point ((i+1)-th point)
                j2 = obj.point(i+1).ts_index(1); % index
                c2 = obj.point(i+1).val;
                t2 = obj.point(i+1).time;

                T = t2 - t1;

                for j = j1 : j2
                    h = (obj.t(j) - t1)/T;

                    % position
                    p(1).val(:, j) = (c2 - c1)*h + c1;
    %                             p(1).val(6, j) = c1(6); % let psi fixed in linear portion

                    % velocity
                    p(2).val(:, j) = (c2 - c1)/T;
    %                             p(2).val(6, j) = 0;

                    % acceleration
%                     p(3).val(:, j) = zeros(6, 1);
                end
            end

            %-2 transition portions
            for i = 2 : obj.num_p - 1 % N points -> N-2 transition points          
                time1 = obj.point(i).time;
                time2 = obj.point(i+1).time;
                tacc = obj.point(i).tacc;
                
                % start point (i-th point - tacc)
                j1 = obj.point(i).ts_index(1); % index
    %                         c1 = p(1).val(6, j1);
                t1 = time1 - tacc;

                % end point (i-th point + tacc)
                j2 = obj.point(i).ts_index(2); % index
    %                         c2 = p(1).val(6, j2);
                t2 = time1 + tacc;

                T = t2 - t1;

                A = p(1).val(:, j1);
                B = obj.point(i).val;
                C = obj.point(i+1).val;
                dA = A - B;
                dC = C - B;

                for j = j1 + 1 : j2 - 1
                    h = (obj.t(j) - t1)/T;
                    CTB = dC*tacc/(time2 - time1) + dA;

                    % position
                    p(1).val(:, j) = (CTB*(2 - h)*h^2 - 2*dA)*h + dA + B;
    %                             obj.path(1).val(6, j) = (c2 - c1)*h + c1; % let psi linearly change in transition portion

                    % velocity
                    p(2).val(:, j) = (CTB*(1.5 - h)*2*h^2 - dA)/tacc;
    %                             obj.path(2).val(6, j) = (c2 - c1)/T;

                    % acceleration
                    p(3).val(:, j) = CTB*(1 - h)*3*h/tacc^2;
                end
            end
        end
    end
end

