function rtadp_wait_release(clock,target)
% No single blocking wait exceeds 10ms; the test remains a wall-clock paced loop.
    remaining=target-toc(clock);
    while remaining>0.002
        pause(min(0.01,remaining-0.001));
        remaining=target-toc(clock);
    end
    while toc(clock)<target
    end
end
