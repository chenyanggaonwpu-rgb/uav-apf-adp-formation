function data=rl_observer_state(observer)
    fields={'Observations','ObservationTimes','Positions','GoalDistances','LastControl','GlobalFeatures'};data=struct();
    for k=1:numel(fields),data.(fields{k})=observer.(fields{k});end
    data.streamState=observer.Stream.State;
end
