function dxdt = numercial_diff(x, h)
[m, n] = size(x);
dxdt = zeros(m, n);

for i = 1 : n - 1
    dxdt(:, i) = (x(:, i+1) - x(:, i))/h;
end
dxdt(:, n) = dxdt(:, n - 1); % suppose dxdt at "len" equal to "len - 1"
end

