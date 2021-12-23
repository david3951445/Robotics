function Print2(rb, joint) % print results of inverse part
th = rad2deg(joint.th)';

fprintf('%5s', '')
for i = 1 : rb.num
    fprintf('%10s', ['theta' num2str(i)])
end
fprintf('%22s', 'out of range joint')
fprintf('\n')

for i = 1 : 8
    if joint.OofR(i)
        str = 'out of reach';
        fprintf('%5d', i)
        fprintf('%16s\n', str)
    else
        % find joints that are out of range
        arr = rb.isOutofRange(th(i, :));
        str = '';
        for j = 1 : length(arr)
            str = append(str, num2str(arr(j)));
            if j ~= length(arr)
                str = append(str, ', ');
            end
        end
    
        fprintf('%5d%10.3f%10.3f%10.3f%10.3f%10.3f%10.3f', i, th(i, :))
        fprintf('%22s\n', str)
    end
end

end