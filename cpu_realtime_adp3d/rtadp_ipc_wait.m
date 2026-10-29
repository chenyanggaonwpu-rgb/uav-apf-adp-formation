function rtadp_ipc_wait(future,timeout)
% Only called before or after a measured flight, never from the 50ms control path.
    timer=tic;
    while ~isfile(future.result)
        if toc(timer)>timeout, error('RTADP:WorkerWaitTimeout','Background process did not respond.'); end
        pause(0.01);
    end
end
