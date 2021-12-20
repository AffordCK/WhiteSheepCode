function [InitialObservation,LoggedSignals] = MyResetFunc(EnvObject)
%   function to reset the env in LoggedSignals

%   reset in the same file
index = unidrnd(EnvObject.MaxIndex);
LoggedSignals.EnvObject = EnvObject.reset('index', index);

[frontCar, behindCar] = LoggedSignals.EnvObject.RoadInstance.get_surrounding_car(1);
State = LoggedSignals.EnvObject.RoadInstance.car_to_state(1, frontCar, behindCar);
LoggedSignals.State = State;
InitialObservation = LoggedSignals.State;

        
end

