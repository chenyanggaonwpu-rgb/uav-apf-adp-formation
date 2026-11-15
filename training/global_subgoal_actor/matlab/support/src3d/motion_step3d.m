function [next,actualAccel] = motion_step3d(state,command,disturbance,p)
% MOTION_STEP3D One synchronous semi-implicit Euler step.
% Command is norm-clipped before the external acceleration is added.
    command=limit_vector3d(command,p.amax);
    actualAccel=command+disturbance;
    next=state;
    next(4:6,:)=limit_vector3d(state(4:6,:)+p.dt*actualAccel,p.vmax);
    next(1:3,:)=state(1:3,:)+p.dt*next(4:6,:);
end
