function [action,mu,logstd]=v4_forward(actor,inputs)
% Pure single-precision inference, shared by MATLAB and compiled Runtime.
    x=single(inputs);assert(size(x,1)==double(actor.input_size));
    layers=0;while isfield(actor,sprintf('W%d',layers+1)),layers=layers+1;end
    for k=1:layers
        y=actor.(sprintf('W%d',k))*x+actor.(sprintf('b%d',k));
        average=mean(y,1);variance=mean((y-average).^2,1);
        y=(y-average)./sqrt(variance+actor.epsilon);
        x=max(single(0),y.*actor.(sprintf('g%d',k))+actor.(sprintf('n%d',k)));
    end
    mu=actor.mean_W*x+actor.mean_b;
    logstd=max(actor.log_std_min,min(actor.log_std_max,actor.logstd_W*x+actor.logstd_b));
    action=tanh(mu);
    assert(all(isfinite(action),'all')&&all(isfinite(logstd),'all'));
end
