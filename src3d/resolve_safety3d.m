function [command,info] = resolve_safety3d(state,command,obstacles,p)
% RESOLVE_SAFETY3D Five-pass bounded predictive repair of swept motion.
% An unresolved hard violation is returned to the caller for explicit failure.
    command=limit_vector3d(command,p.amax);
    original=command;
    info.iterations=0;
    if p.safety.enable
        for pass=1:p.safety.maxIterations
            predicted=motion_step3d(state,command,zeros(3,p.nUAV),p);
            correction=zeros(size(command));
            for i=1:p.nUAV
                start=state(1:3,i);finish=predicted(1:3,i);
                for k=1:numel(obstacles)
                    samples=[start,(start+finish)/2,finish];
                    distances=zeros(1,3);normals=zeros(3,3);
                    for s=1:3
                        [distances(s),normals(:,s)]= ...
                            obstacle_distance3d(samples(:,s),obstacles(k));
                    end
                    [d,j]=min(distances);
                    threshold=p.obsSafe+p.safety.margin;
                    if d<threshold
                        normal=normals(:,j);
                        closing=min(0,dot(predicted(4:6,i),normal));
                        correction(:,i)=correction(:,i)+ ...
                            p.collisionAvoidGain*(threshold-d)*normal- ...
                            p.collisionVelGain*closing*normal;
                    end
                end
            end
            for i=1:p.nUAV-1
                for j=i+1:p.nUAV
                    r0=state(1:3,i)-state(1:3,j);
                    dr=(predicted(1:3,i)-predicted(1:3,j))-r0;
                    t=max(0,min(1,-dot(r0,dr)/max(dot(dr,dr),eps)));
                    vector=r0+t*dr;
                    d=norm(vector);
                    threshold=p.uavSafe+p.safety.margin;
                    if d<threshold
                        if d<1e-9
                            normal=[cos(2*pi*i/p.nUAV);sin(2*pi*i/p.nUAV);0];
                        else
                            normal=vector/d;
                        end
                        closing=min(0,dot(predicted(4:6,i)- ...
                            predicted(4:6,j),normal));
                        push=p.collisionAvoidGain*(threshold-d)*normal- ...
                            p.collisionVelGain*closing*normal;
                        correction(:,i)=correction(:,i)+0.5*push;
                        correction(:,j)=correction(:,j)-0.5*push;
                    end
                end
            end
            lo=p.workspace(1,:)';hi=p.workspace(2,:)';
            for i=1:p.nUAV
                predictedPosition=predicted(1:3,i);
                below=max(lo+p.obsSafe+p.safety.margin-predictedPosition,0);
                above=max(predictedPosition-(hi-p.obsSafe-p.safety.margin),0);
                correction(:,i)=correction(:,i)+ ...
                    p.collisionAvoidGain*(below-above);
            end
            info.iterations=pass;
            if max(vecnorm(correction))<1e-10, break; end
            command=limit_vector3d(command+correction,p.amax);
        end
    end
    predicted=motion_step3d(state,command,zeros(3,p.nUAV),p);
    [hit,info.minObstacle,info.minPair,info.boundaryHit]= ...
        check_swept3d(state,predicted,obstacles,p);
    info.predictedSafe=~hit;
    info.triggered=max(vecnorm(command-original))>1e-8;
    info.correctionNorm=norm(command-original,'fro');
end
