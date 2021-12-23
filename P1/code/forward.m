function T6 = forward(rb, theta) % forward (joints -> noap)
T6 = eye(4); % noap matrix
for i = 1 : rb.num
    T6 = T6*rb.A(i, theta(i)/180*pi); % multiply from A1 to A6
end
end