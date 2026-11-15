function rl_restore_observer(observer,data)
    fields={'Observations','ObservationTimes','Positions','GoalDistances','LastControl','GlobalFeatures'};
    for k=1:numel(fields),observer.(fields{k})=data.(fields{k});end
    observer.Stream.State=data.streamState;
end
