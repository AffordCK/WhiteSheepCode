classdef Env
    %   ENV class
    %   include road(env) and vehicle set
    % 
    
    properties
        RoadInstance                                % the road instance
        PlannerInstance                             % the planner instance
        RewardInstance                              % the reward instance
        Conf                                        % the configuration information
        
        FileName                                    % scenario file name
        VehicleList                                 % the vehicle list in one data file
        CellIndex                                   % the index of the cell
        
    end
    
    methods
        function obj = Env()
            fprintf("[%s]: Creating Env object...\n", datestr(now));
            obj.PlannerInstance = Planner();
            obj.RewardInstance = Reward();
            fprintf("[%s]: Loding configuration info...\n", datestr(now));
            obj.Conf = conf();
        end
        
        function [obj, vehicleSet, vehiclesTrajectory] = load_scenario(obj, filename, index)
            %   load scenario data from file
            %   param: filename, the file name of the data file
            %   param: index, the scenario index in the scenario
            %   output: vehicleSet, the set of vehicle initial state
            %   output: vehilesTrajectory, the trajectory of env Car
            %   Warnning: output stay in global frame, need transforming
            
            fprintf("[%s]: Loading Scenario Data from %s...\n", datestr(now), filename);
            
            obj.FileName = filename;
            obj.CellIndex = index;
            vehicleList = load(filename, 'vehicleList').vehicleList;
            obj.VehicleList = vehicleList;
            vehiclesTrajectory = vehicleList{1, index}.vehicles;
            vehicleNum = 1 + numel(vehiclesTrajectory);
            vehicleSet = zeros(vehicleNum, 3);
            vehicleSet(1, 1) = vehicleList{1, index}.startPoint(1);     % egoCar start X
            vehicleSet(1, 2) = vehicleList{1, index}.startPoint(2);     % egoCar start Y
            vehicleSet(1, 3) = vehicleList{1, index}.startPoint(3);     % egoCar start longitudinal velocity
            
            for i = 2:vehicleNum
                vehicleSet(i, 1) = vehiclesTrajectory{1, i - 1}.X(1);   % envCar start X
                vehicleSet(i, 2) = vehiclesTrajectory{1, i - 1}.Y(1);   % envCar start Y

                vehicleSet(i, 3) = vehiclesTrajectory{1, i - 1}.Yaw(1); % envCar start Yaw
                
                % handl nan in Yaw
                nanIndex = find(isnan(vehiclesTrajectory{1, i - 1}.Yaw));
                vehiclesTrajectory{1, i - 1}.Yaw(nanIndex) = zeros(size(nanIndex));
                
                xDiff = diff(vehiclesTrajectory{1, i - 1}.X);
                xDiff(end + 1) = xDiff(end);
                xDiff = xDiff / obj.Conf.TimeResolution;
                
                vehiclesTrajectory{1, i - 1} = [vehiclesTrajectory{1, i - 1}.X,...
                        vehiclesTrajectory{1, i - 1}.Y,...
                        vehiclesTrajectory{1, i - 1}.Yaw,...
                        xDiff];
            end
            
        end
        
        function obj = load_road(obj, waypoint, vehicleset, vehiclestrajectory)
            %   load RoadInstance from vehicleset and vehiclestrajectory
            %   param: waypoint, the center point of the way
            %   param: vehicleset, the set of vehicle
            %   param: vehiclestrajectory, the trajectory of env vehicle
            %   warnning: input stay in global frame
            
            fprintf("[%s]: Loading road data...\n", datestr(now));
            
            obj.RoadInstance = Road(waypoint);
            vehicleset(:, 1:2) = obj.RoadInstance.global_to_frenet_trajectory(vehicleset(:, 1:2));
            obj.RoadInstance = obj.RoadInstance.add_vehicles(vehicleset, vehiclestrajectory);
        end
        
        function obj = reset(obj, varargin)
            %   function to reload Road Instance
            %   param: filename, the scenario file name
            %   param: index, the cell index in the file
            
            fprintf("[%s]: Env object reseting...\n", datestr(now));
            
            param = inputParser;
            addParameter(param, 'filename', obj.FileName);
            addParameter(param, 'index', obj.CellIndex);
            param.parse(varargin{:});
            
            obj.RoadInstance = obj.RoadInstance.reset();
            
            fileName = param.Results.filename;
            cellIndex = param.Results.index;
            
            if ~exist(fileName, 'file')
                fileName = "Scenario/04.mat";
            end
            
            if cellIndex <= 0 || cellIndex > numel(obj.VehicleList)
                cellIndex = 1;
            end
            
            if strcmp(fileName, obj.FileName)
                vehiclesTrajectory = obj.VehicleList{1, cellIndex}.vehicles;
                vehicleNum = 1 + numel(vehiclesTrajectory);
                vehicleSet = zeros(vehicleNum, 3);
                vehicleSet(1, 1) = obj.VehicleList{1, cellIndex}.startPoint(1);     % egoCar start X
                vehicleSet(1, 2) = obj.VehicleList{1, cellIndex}.startPoint(2);     % egoCar start Y
                vehicleSet(1, 3) = obj.VehicleList{1, cellIndex}.startPoint(3);     % egoCar start longitudinal velocity

                for i = 2:vehicleNum
                    vehicleSet(i, 1) = vehiclesTrajectory{1, i - 1}.X(1);   % envCar start X
                    vehicleSet(i, 2) = vehiclesTrajectory{1, i - 1}.Y(1);   % envCar start Y

                    vehicleSet(i, 3) = vehiclesTrajectory{1, i - 1}.Yaw(1); % envCar start Yaw

                    % handl nan in Yaw
                    nanIndex = find(isnan(vehiclesTrajectory{1, i - 1}.Yaw));
                    vehiclesTrajectory{1, i - 1}.Yaw(nanIndex) = zeros(size(nanIndex));

                    xDiff = diff(vehiclesTrajectory{1, i - 1}.X);
                    xDiff(end + 1) = xDiff(end);
                    xDiff = xDiff / obj.Conf.TimeResolution;

                    vehiclesTrajectory{1, i - 1} = [vehiclesTrajectory{1, i - 1}.X,...
                            vehiclesTrajectory{1, i - 1}.Y,...
                            vehiclesTrajectory{1, i - 1}.Yaw,...
                            xDiff];
                end
            else
                [obj, vehicleSet, vehiclesTrajectory] = obj.load_scenario(fileName, cellIndex);
            end
            
            vehicleSet(:, 1:2) = obj.RoadInstance.global_to_frenet_trajectory(vehicleSet(:, 1:2));
            obj.RoadInstance = obj.RoadInstance.add_vehicles(vehicleSet, vehiclesTrajectory);
            
        end
        
        function action = sample_action(obj, state)
            %   function to sample action from network
            %   param: state, the state of the egoCar 
            %   output: action, [delta_s, delta_s_dot, delta_s_ddot, delta_l, delta_l_dos, delta_l_ddos, delta_T]
            
            
        end
        
        function [obj, reward, newState, Done] = step(obj, action)
            %   function to take action from  sampe
            %   param: action, [delta_s, delta_s_dot, delta_s_ddot, delta_l, delta_l_dos, delta_l_ddos, delta_T]
            
            %   Planning Firstly
            planStartState = obj.RoadInstance.VehicleSet{1, 1}.FrenetState;
            planResult = obj.PlannerInstance.get_trajectory(planStartState,...
                planStartState + action(1:end - 1),...
                action(end),...
                obj.Conf.TimeResolution,...
                1);
            planTrajectory = planResult.trajectory;
            
            %   Check the feasiblity
            newState = zeros(1, 14);
            if obj.PlannerInstance.check_trajectory(planTrajectory)
%                 newState(1) = planStartState(1) + action(1);
%                 newState(2) = -10;
                reward = obj.RewardInstance.InFeasibleReward;
                Done = true;
                return;
            end
            
            %   Check lane boundary
            if max(planTrajectory(:, 4)) >= max(obj.RoadInstance.LaneBoundary)
%                 newState(1:2) = planStartState(1:2) + action(1:2);
%                 newState(7:8) = [obj.Conf.ObserScope, 0];
%                 newState(13:14) = [-obj.Conf.ObserScope, 0];
                reward = obj.RewardInstance.InFeasibleReward;
                Done = true;
                return;
            elseif min(planTrajectory(:, 4)) <= min(obj.RoadInstance.LaneBoundary)
%                 newState(1:2) = planStartState(1:2) + action(1:2);
%                 newState(3:4) = [obj.Conf.ObserScope, 0];
%                 newState(9, 10) = [-obj.Conf.ObserScope, 0];
                reward = obj.RewardInstance.InFeasibleReward;
                Done = true;
                return;
            end
            
            globalTrajectory = obj.RoadInstance.frenet_to_global_trajectory(planTrajectory);
            
            % Excute the trajectory
            [obj.RoadInstance, result] = obj.RoadInstance.move_along_trajectory(globalTrajectory,...
                obj.RoadInstance.CurrentTimeIndex);
            
            if ~result.flag
                %   collision happen
                reward = obj.RewardInstance.CollisionReward;
                Done = true;
                return;
            end
            
            if result.finish
                %   finish the 
                Done = true;
            end
            
            %   Calculate the reward
            finishTrajectory = planTrajectory(1: result.runTimeIndex, :);
            obj.RewardInstance = obj.RewardInstance.cal_all_reward(finishTrajectory);
            obj.RewardInstance = obj.RewardInstance.cal_total_reward();
            reward = obj.RewardInstance.TotalReward;
            
            %   Update the egoCar Frenet State
            obj.RoadInstance.VehicleSet{1, 1}.FrenetState = planTrajectory(result.runTimeIndex, :);
            
            %   Get the new State from DRL
            [frontCar, behindCar] = obj.RoadInstance.get_surrounding_car(1);
            newState = obj.RoadInstance.car_to_state(1, frontCar, behindCar);
            
        end
        
        
    end
end

