function holdTime = update_success_hold(holdTime, conditionsMet, dt)
% UPDATE_SUCCESS_HOLD 仅在全部成功条件连续满足时累计保持时间。
    if conditionsMet
        holdTime = holdTime + dt;
    else
        holdTime = 0;
    end
end
