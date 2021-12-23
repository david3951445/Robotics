function Print1(T) % print results of forward part
% find ϕ, θ, ψ (z-y-z Euler angle)
ax = T(1, 3); ay = T(2, 3); az = T(3, 3);
nx = T(1, 1); ny = T(2, 1);
if ax == 0 && ay == 0
    % In this case, theta = 0 i.e. θ and ψ correspond to the same rotating axis 
    phi = atan2(ny, nx); % because we set psi as zero
    theta = 0;
    psi = 0;
else % theta ~= 0
    phi = atan2(ay, ax);
    theta = atan2((cos(phi)*ax + sin(phi)*ay), az);
    psi = atan2(-sin(phi)*nx + cos(phi)*ny, -sin(phi)*T(1, 2) + cos(phi)*T(2, 2));
end


% print (n, o, a, p)
fprintf('%10s', ["n" "o" "a" "p"])
fprintf('\n')
for i = 1 : 4
    fprintf('%10.3f%10.3f%10.3f%10.3f\n', T(i, :))
end
fprintf('\n')

% print (x, y, z, ϕ, θ, ψ)
fprintf('%10s', ["x" "y" "z" "phi", "theta", "psi"])
fprintf('\n')
fprintf('%10.3f%10.3f%10.3f%10.3f%10.3f%10.3f\n', T(1:3, 4), rad2deg([phi theta psi]))
fprintf('\n')

end