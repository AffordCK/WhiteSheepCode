function [Observation, Reward, IsDone, LoggedSignals] = MyStepFunc(Action, LoggedSignals)
%   function to take a step in rl env

[LoggedSignals.EnvObject, Observation, Reward, IsDone] = LoggedSignals.EnvObject.step(Action);

LoggedSignals.State = Observation;

end

