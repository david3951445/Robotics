classdef p
    %P Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        a
    end
    
    methods
        function obj = p(x)
            obj.a = obj.f(x);
        end
    end
    
    methods (Access = private)
        function y = f(~, x)
            y = x;
        end
    end
end

