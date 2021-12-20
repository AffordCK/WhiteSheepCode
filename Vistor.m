classdef Vistor
    %VISTOR Class
    %   used to show the dynamic picture
    
    properties
        Scenario            % the driving scenario
        EgoCar              % the ego car
        EnvCar              % the env car cell
    end
    
    methods
        function obj = Vistor(waypoints, lanenum, VehicleSet)
            %   vistor constructor function
            %   param: waypoints, the road center points
            %   param: lanenum, the numble of the lane
            %   param: VehicleSet, the set of vehicle
            fprintf("[%s]: Creating Vistor object...\n", datestr(now));
            
            obj.Scenario = drivingScenario;
            roadCenters = [waypoints, zeros(size(waypoints, 1), 1)];
            laneSpecification = lanespec(lanenum);
            road(obj.Scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');
            obj.EgoCar = vehicle(obj.Scenario, ...
                'ClassID', 1,...
                'Position', [VehicleSet{1, 1}.GlobalState(1), VehicleSet{1, 1}.GlobalState(2), 0],...
                'Mesh', driving.scenario.carMesh, ...
                'Name', 'Car');
            obj.EnvCar = {};
            for i = 2: numel(VehicleSet)
                obj.EnvCar{i - 1} = vehicle(obj.Scenario, ...
                    'ClassID', 1, ...
                    'Position', [VehicleSet{1, i}.GlobalState(1), VehicleSet{1, i}.GlobalState(2), 0], ...
                    'Yaw', VehicleSet{1, i}.Yaw,...
                    'Mesh', driving.scenario.carMesh, ...
                    'Name', 'Car2');
            end
        end
    end
end

