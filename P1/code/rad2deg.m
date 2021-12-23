function y = rad2deg(th) % convert rad to -180~180 degree
y = th/pi*180;

% convert the value to -180~180
[n, m] = size(th);
for i = 1 : n
    for j = 1 : m
        if y(i, j) > 180
            y(i, j) = y(i, j) - 360;
        end
        if y(i, j) < -180
            y(i, j) = y(i, j) + 360;
        end
    end
end
end