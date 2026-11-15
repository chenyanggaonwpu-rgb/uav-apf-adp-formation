classdef V4Legacy < handle
    properties
        Net
    end
    methods
        function obj=V4Legacy(net),obj.Net=net;end
        function action=predict(obj,observation)
            x=single(observation);n=obj.Net;
            action=double(tanh(n.W3*max(single(0),n.W2*max(single(0),n.W1*x+n.b1)+n.b2)+n.b3));
        end
    end
end
