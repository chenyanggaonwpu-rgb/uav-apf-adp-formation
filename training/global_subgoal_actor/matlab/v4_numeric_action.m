function action=v4_numeric_action(actor,observation)
    assert(isequal(size(observation),[32 1])&&all(isfinite(observation)));
    action=double(predict(actor,single(observation)));
    assert(isequal(size(action),[3 1])&&all(isfinite(action)));
end
