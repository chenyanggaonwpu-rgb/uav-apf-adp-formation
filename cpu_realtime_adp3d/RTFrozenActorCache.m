classdef RTFrozenActorCache < handle
% Adapter consumed by the original frozen supervisor. It holds an action, not
% neural weights. The real immutable Actor runs only in the CPU process worker.
    properties
        Action=zeros(3,1)
        LastObservation=zeros(32,1)
        HasObservation=false
    end
    methods
        function output=predict(cache,input)
            cache.LastObservation=double(extractdata(input));
            cache.HasObservation=true;
            assert(isequal(size(cache.LastObservation),[32 1]));
            output=dlarray(single(cache.Action),'CB');
        end
    end
end
