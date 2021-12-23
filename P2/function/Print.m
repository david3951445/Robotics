function Print(traj, method)
str1 = ["position(m)" "velocity(m/s)" "acceleration(m/s^2)"
    "angle(degree)" "angle velocity(degree/s)" "angle acceleration(degree/s^2)"];
str2 = ["x" "y" "z" "phi" "theta" "psi"];

%% plot configuration
n = 3; % n = 1 ~ 3
for i = 1 : n % pos, vel, acc  
    Traj = traj.path(i).config;
    t = traj.t;
    
    for j = 1 : 1 % (x, y, z), (phi, theta, psi)
        figure
        for k = 1 : 3
            k1 = 3*(j-1) + k; % j1 = 1 ~ 6
            
            subplot(3, 1, k)
            
            if j == 1 % (x, y, z)
                tr = Traj(k1, :);
            else % (phi, theta, psi)
                tr = rad2deg(Traj(k1, :)); % rad -> degree
            end
            
            plot(t, tr);
            title(str2(k1));
            xlabel('t');
            ylabel(str1(j, i));
            %             ytickformat('%.1f') % ylabel precision
        end
    end
end

%% plot joint
for i = 1 : n
    Traj = traj.path(i).joint;
    t = traj.t;
    
    figure
    for j = 1 : size(Traj, 1)
        subplot(3, 2, j)
        
        tr = rad2deg(Traj(j, :)); % rad -> degree
        
        plot(t, tr);
        title(['joint ' num2str(j)]);
        xlabel('t');
        ylabel(str1(2, i));
        %             ylim([-180 180])
    end
end

%% plot 3D path
% construct X, Y, Z, U, V, W for quiver3(X, Y, Z, U, V, W)
p = zeros(3, traj.len_t); % X, Y, Z (position)
a = zeros(3, traj.len_t); % U, V, W (orientation vector)
for i = 1 : traj.len_t
    noap = traj.path(1).noap;
    a(:, i) = noap(1:3, 3, i); % noap's z-axis == noap's a
    p(:, i) = noap(1:3, 4, i);
end

% orientation of path
figure
quiver3(p(1, :), p(2, :), p(3, :), a(1, :), a(2, :), a(3, :), 'g'); 
hold on

% position of path
plot3(p(1, :), p(2, :), p(3, :), 'Color', 'b', 'LineWidth', 3); 

% desired points of path
for i = 1 : 3 % P1 ~ P3
    P = traj.point(i).noap;
    plot3(P(1, 4), P(2, 4), P(3, 4), 'o');
    txt = ['P' num2str(i) '(' num2str(P(1, 4)) ',' num2str(P(2, 4)) ',' num2str(P(3, 4)) ')'];
    text(P(1, 4), P(2, 4), P(3, 4), txt, 'VerticalAlignment','bottom','HorizontalAlignment','right');
    
    % P's coordinate system
    quiver3(P(1, 4), P(2, 4), P(3, 4), P(1, 1), P(2, 1), P(3, 1), 0.1, 'Color', 'b', 'LineWidth', 1.5); % x-axis
    quiver3(P(1, 4), P(2, 4), P(3, 4), P(1, 2), P(2, 2), P(3, 2), 0.1, 'Color', 'r', 'LineWidth', 1.5); % y-axis
    quiver3(P(1, 4), P(2, 4), P(3, 4), P(1, 3), P(2, 3), P(3, 3), 0.1, 'Color', 'g', 'LineWidth', 1.5); % z-axis
end

title(['3D path of ' method ' Move'])
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');

hold off
end