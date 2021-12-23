    function joint = inverse(rb, T6) % inverse dynamic (noap -> joints)
th = zeros(rb.num, 8); % joints answer
OofR = zeros(1, 8); % out of reach judgment, bool values

px = T6(1, 4); py = T6(2, 4); pz = T6(3, 4);

% theta 1 
if px == 0 && py == 0 % (if a1 + a2c2 + a3c23 == 0)
    th(1, 1:2) = [-pi/2, pi/2]; % default value, in fact, theta 1 satisfied -150 <= theta 1 <= 150
else
    th(1, 1:2) = [atan2(py, px) atan2(py, px) + pi];
end

for i = 1 : 2
    index = (1:2) + 2*(i-1); % just for indexing
    
    % follow the steps in the handout
    f1 = px*cos(th(1, i)) + py*sin(th(1, i)) - rb.a(1);
    f2 = -pz;
     
    % theta 2
    % g1, g2 are algebraic substitutions
    g1 = f1/f2;
    g2 = (f1^2 + f2^2 + rb.a(2)^2 - rb.a(3)^2)/(2*rb.a(2)*f2);
    
    % c2 denotes cos(theta 2), s2 denotes sin(theta 2)
    c2 = (g1*g2 + sqrt(g1^2 - g2^2 + 1))/(1 + g1^2); % Quadratic formula
    s2 = g2 - g1*c2;
    c2_ = (g1*g2 - sqrt(g1^2 - g2^2 + 1))/(1 + g1^2);
    s2_ = g2 - g1*c2_;
    
    try
        th(2, index) = [atan2(s2, c2) atan2(s2_, c2_)];
    catch % there is no solution -> out of reach
        OofR(index) = 1;
    end
    
    % theta 3
    c3 = (f1^2 + f2^2 - rb.a(2)^2 - rb.a(3)^2)/(2*rb.a(2)*rb.a(3));
    s3 = sqrt(1 - c3^2);
    
    try
        th(3, index) = [atan2(s3, c3) atan2(-s3, c3)];
    catch % there is no solution -> out of reach
        % OofR(index) = 1;
    end
    
    % determine the arrangement of theta 2 and theta 3 from the equation "f1 = a2c2 + a3c23"
    % if (f1 - a2c2 - a3c23 != 0)
    if norm(f1 - rb.a(2)*cos(th(2, index)) - rb.a(3)*cos(th(2, index) + th(3, index))) > 10^(-4)
        th(2, index) = th(2, flip(index)); % rearrange
    end
end

th(1, 1:4) = repelem(th(1, 1:2), 2); % arrange theta 1

for i = 1 : 4
    T3 = eye(4);
    for j = 1 : 3
        T3 = T3*rb.A(j, th(j, i));
    end
    T36 = T3\T6; % remove T3 (since it is known)

    % theta 4 (if s5 != 0)
    ax = T36(1, 3); ay = T36(2, 3);
    if ax == 0 && ay == 0
        th(4, (1:2) + 2*(i-1)) = [-pi/2, pi/2]; % default value, in fact, -110 <= theta 4 <= 110
    else
        th(4, (1:2) + 2*(i-1)) = [atan2(ay, ax) atan2(ay, ax) + pi];
    end
end

th(1:3, :) = repelem(th(1:3, 1:4), 1, 2); % arrange theta 1~3
OofR = repelem(OofR(1:4), 1, 2); % arrange OofR

for i = 1 : 8
    T4 = eye(4);
    for j = 1 : 4
        T4 = T4*rb.A(j, th(j, i));
    end
    T46 = T4\T6; % remove T4
    
    % theta 5
    th(5, i) = atan2(T46(1, 3), -T46(2, 3));
    
    % theta 6
    th(6, i) = atan2(T46(3, 1), T46(3, 2));
end

joint.th = th; joint.OofR = OofR; % return ans
end