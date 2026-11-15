function [stagnant,revisit]=rl_motion_metrics(result)
    dt=result.params.dt;points=[result.initialFleetState(1:3,1),squeeze(result.history.state(1:3,1,:))];
    n=size(points,2)-1;stationary=false(1,n);returns=false(1,n);window=round(2/dt);separation=round(8/dt);
    cells=floor(points/3);last=containers.Map('KeyType','char','ValueType','double');
    for k=1:n
        if k>window,stationary(k)=max(vecnorm(points(:,k-window:k+1)-points(:,k+1)))<.25;end
        key=sprintf('%d,%d,%d',cells(:,k+1));
        if isKey(last,key)&&k-last(key)>=separation,returns(k)=true;end
        last(key)=k;
    end
    stagnant=dt*nnz(stationary);revisit=dt*nnz(returns);
end
