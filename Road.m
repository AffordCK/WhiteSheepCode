classdef Road
    %   ROAD class 
    %   implement global <-> frenet frame transform
    
    properties
%         FrenetRefPath           % the frenet reference of the road [s, s_dot, s_ddot, l, l_dos, l_ddos]
        refPath                   % the reference of the road {referencePathFrenet}, lane id [LaneNum, LaneNum - 1, ..., 1]
        LaneBoundary              % the boundary of lane in frenet coordinate
        StaticMap                 % the map between VehicleSet and VehicleTrajectory
        CurrentTimeIndex          % the current trajectory time index
        VehicleSet                % the set of vehicle [vehicle1(EgoVehicle), vehicle2, ..., Vehiclen]
        VehicleTrajectory         % the trajectory of envCar in global frame
%         EgoVehicle              % the ego vehicle
        VistorInstance            % the vistor
%         Planner = Planner()     % the planner
        Conf                      % configuration
    end
    
    methods
        function obj = Road(waypoint)
            %   ROAD construct function
            %   param: waypoint, the center point of the road

            %   set the right road boundary as the reference path
            
            fprintf("[%s]: Creating Road object...\n", datestr(now));
            
            obj.Conf = conf();
            
            refPath = referencePathFrenet(waypoint);
            globalState = zeros(size(refPath.Waypoints, 1), 6);
            globalState(:, 1:2) = refPath.Waypoints;
            frenetState = global2frenet(refPath, globalState);
            frenetState(:, 4) = -obj.Conf.LaneNum * obj.Conf.LaneWidth / 2;   % calculate the right most lane boundary
            GlobalRefPath = frenet2global(refPath, frenetState);
            obj.refPath = referencePathFrenet(GlobalRefPath(:, 1:2));      
            % calculate the lane boundary
            obj.LaneBoundary = [0: obj.Conf.LaneWidth: obj.Conf.LaneWidth * obj.Conf.LaneNum];
            obj.CurrentTimeIndex = 1;
        end
        
        function obj = add_vehicles(obj, vehicleset, vehiclestrajectory)
            %   function add vehicles to road
            %   param: vehicleset, the set of vehicle [s1, d1, v1;
            %                                   s2, d2, yaw2;
            %                                   ...;
            %                                   sn, dn, yawn;]
            %   param: vehiclestrajectory, the trajectory of env car 
            %                                   [x1, y1, yaw1;
            %                                   x2, y2, yaw2;
            %                                   ...;
            %                                   xn, yn, yawn;;]...
            
            % initiate the ego vehicle
            
            fprintf("[%s]: Adding Vehicles object...\n", datestr(now));
            
            obj.VehicleSet{1, 1} = Vehicle(vehicleset(1, 1:2),...
                'longvelocity', vehicleset(1, 3));
            % initiate the env vehicle
            for vehicleIndex = 2:size(vehicleset, 1)
                obj.VehicleSet{1, vehicleIndex} = Vehicle(vehicleset(vehicleIndex, 1:2), 'yaw', vehicleset(vehicleIndex, 3));
            end
            % transform frenet into global
            for i = 1:numel(obj.VehicleSet)
                obj.VehicleSet{1, i}.GlobalState = frenet2global(obj.refPath,...
                    obj.VehicleSet{1, i}.FrenetState);
            end
            obj.VehicleTrajectory = vehiclestrajectory;
            obj.StaticMap = containers.Map([2: size(vehicleset, 1)], [1: numel(vehiclestrajectory)]);
        end
        
        function obj = clear_vehicles(obj)
            obj.VehicleSet = {};
            obj.VehicleTrajectory = {};
            obj.StaticMap = [];
        end
        
        function obj = add_vistor(obj, waypoint)
            %   function add vistor to road
            fprintf("[%s]: Adding Vistor object...\n", datestr(now));
            obj.VistorInstance = Vistor(waypoint, obj.Conf.LaneNum, obj.VehicleSet);
        end
        
        function obj = clear_vistor(obj)
            obj.VistorInstance = [];
        end
        
        function obj = reset(obj)
            %   reset function
            obj.CurrentTimeIndex = 1;
            obj = obj.clear_vehicles();
            obj = obj.clear_vistor();
        end
        
        function laneNum = get_current_lane(obj, frenetposition)
            %   get the lane numble in frenet position
            %   param: frenet position [s, l] in frenet coordinate
            %   return: the lane numble
            laneNum = ceil(frenetposition(2) / obj.Conf.LaneWidth);
        end
        
        function frenetState = get_vehicle_frenetstate(obj, vehicleid)
            %   function to get the frenet state of vehicleid
            %   output: frenetState, used to plan the trajectory
            frenetState = obj.VehicleSet{1, vehicleid}.FrenetState;
        end
        
        function [frontCar, backCar] = get_surrounding_car(obj, vehicleid)
            %   get the surrounding vehicle around vehicleid.
            %   param: vehicleid, the id of the vehicle
            frenetState = obj.VehicleSet{1, vehicleid}.FrenetState;
            s = frenetState(1);
            d = frenetState(4);
            laneId = get_current_lane(obj, [s, d]);
            %   obj.VehicleSet{1, vehicleid}.Lane = laneId;
            frontCar = {[], [], []};         % left mid right (inf [] vehicle) -> (out of the lane, no car, vehicle)
            backCar = {[], [], []};         % left mid right (inf [] vehicle) -> (out of the lane, no car, vehicle)
            
            for laneIndex = -1:1
               switch (laneIndex)
                   case 1
                        % find the vehicle left to position in d direction
                        [row, col] = find(cellfun(@(x) (d - x.FrenetState(4) <...
                            -obj.Conf.LaneWidth / 2 && d - x.FrenetState(4) >= (-1.5 * obj.Conf.LaneWidth)), obj.VehicleSet));
                   case 0
                        % find the vehicle near to position in d direction
                        [row, col] = find(cellfun(@(x) abs(x.FrenetState(4) - d) <=...
                            obj.Conf.LaneWidth / 2, obj.VehicleSet));
                   case -1
                        % find the vehicle right to position in d direction
                        [row, col] = find(cellfun(@(x) (d - x.FrenetState(4) >...
                            obj.Conf.LaneWidth / 2 && d - x.FrenetState(4) <= (1.5 * obj.Conf.LaneWidth)), obj.VehicleSet));
               end
               if laneId + laneIndex > obj.Conf.LaneNum || laneId + laneIndex <= 0
                    % TODO: find a better way to check whether the car is
                    % out of the road.
                    % this may product error
                    frontCar{1, -laneIndex + 2} = inf;      % need modifying
                    backCar{1, -laneIndex + 2} = -inf;
               elseif numel(row) == 0   
                   % if there is no car nearby and no road out, it should never
                   % happen, because of egoCar itself
                   continue;
               else
                    % find the vehicle near to position in s direction
                    frontS = inf;
                    behindS = -inf;
                    for index = 1:numel(row)
                        i = row(index);
                        j = col(index);
                        if j == vehicleid
                            continue
                        end
                        relatDist = obj.VehicleSet{i, j}.FrenetState(1) - s;
                        if relatDist >= 0 && relatDist < obj.Conf.ObserScope && relatDist < frontS
                            frontS = relatDist;
                            frontCar{1, -laneIndex + 2} = obj.VehicleSet{i, j};
                        elseif relatDist < 0 && relatDist > -obj.Conf.ObserScope && relatDist > behindS
                            behindS = relatDist;
                            backCar{1, -laneIndex + 2} = obj.VehicleSet{i, j};
                        end
                    end
               end
            end
        end
        
        function state = car_to_state(obj, vehicleid, frontCar, behindCar)
            %   function to get the state: [egoCarS,    egoCarV,        (1, 2)
            %                               leftLeadS,  leftLeadV,      (3, 4)
            %                               leadS,      leadV,          (5, 6)
            %                               rightLeadS, rightLeadV,     (7, 8)
            %                               leftFolS,   leftFolV,       (9, 10)
            %                               folS,       folV,           (11, 12)
            %                               rightFolS,  rightFolV]      (13, 14)
            %   TODO: lane info like curvature may be included.
            state = zeros(14, 1);
            state(1) = obj.VehicleSet{1, vehicleid}.FrenetState(1);     % may be we don't need this
            state(2) = obj.VehicleSet{1, vehicleid}.FrenetState(2);
            
            % transform the leading car state
            for carIndex = 1: 3
                if numel(frontCar{1, carIndex}) == 0
                    state(2 * carIndex + 1) = obj.Conf.ObserScope;
                    state(2 * carIndex + 2) = 0;
                elseif isa(frontCar{1, carIndex}, 'double')
                    state(2 * carIndex + 1) = 0;
                    state(2 * carIndex + 2) = 0;
                else
                    state(2 * carIndex + 1) = frontCar{1, carIndex}.FrenetState(1) - ...
                         obj.VehicleSet{1, vehicleid}.FrenetState(1);
                    state(2 * carIndex + 2) = frontCar{1, carIndex}.FrenetState(2) - ...
                         obj.VehicleSet{1, vehicleid}.FrenetState(2);
                end
            end
            
            % transform the following car state
            for carIndex = 1:3
                if numel(behindCar{1, carIndex}) == 0
                    state(2 * carIndex + 7) = -obj.Conf.ObserScope;
                    state(2 * carIndex + 8) = 0;
                elseif isa(behindCar{1, carIndex}, 'double')
                    state(2 * carIndex + 7) = 0;
                    state(2 * carIndex + 8) = 0;
                else
                    state(2 * carIndex + 7) = behindCar{1, carIndex}.FrenetState(1) - ...
                        obj.VehicleSet{1, vehicleid}.FrenetState(1);
                    state(2 * carIndex + 8) = behindCar{1, carIndex}.FrenetState(2) - ...
                         obj.VehicleSet{1, vehicleid}.FrenetState(2);
                end
            end
%             state = state(2:end);
            
        end
        
        function flag = check_collision(obj, vehicleid)
            %   function to collision detection in global coordinate
            %   param: vehicleid, the center car id
            %   output: flag, False when collision happen
            flag = true;
            delta = 0.1;    % safe boundary
            l = obj.VehicleSet{1, vehicleid}.Length / 2 + delta;
            w = obj.VehicleSet{1, vehicleid}.Width / 2 + delta;
            %   the max radius check collision
            maxDist = 2 * sqrt(w^2 + l^2);
            x0 = obj.VehicleSet{1, vehicleid}.GlobalState(1);
            y0 = obj.VehicleSet{1, vehicleid}.GlobalState(2);
            %   find the surrounding car laying on the circle
            [row, col] = find(cellfun(@(x) sqrt((x0 - x.GlobalState(1))^2 + ...
                (y0 - x.GlobalState(2))^2) <= maxDist, obj.VehicleSet));
            if numel(row) == 1 && numel(col) == 1
                return;
            end
            %   use vector implement projection
            yaw = obj.VehicleSet{1, vehicleid}.Yaw;
            position = [[x0, y0] + l * [cos(yaw), sin(yaw)] + w * [cos(yaw - pi / 2), sin(yaw - pi / 2)];...
                [x0, y0] + l * [cos(yaw), sin(yaw)] + w * [cos(yaw + pi / 2), sin(yaw + pi / 2)];...
                [x0, y0] + l * [cos(yaw - pi), sin(yaw - pi)] + w * [cos(yaw + pi / 2), sin(yaw + pi / 2)];...
                [x0, y0] + l * [cos(yaw - pi), sin(yaw - pi)] + w * [cos(yaw - pi / 2), sin(yaw - pi / 2)]];
            basePosition = [position(4, :); position(1, :); position(2, :); position(3, :);];
            vector = position - basePosition;
%             vector = [position(1, :) - position(4, :); position(2, :) - position(1, :);...
%                 position(3, :) - position(2, :); position(3, :) - position(4, :)];
            
            for index = col
                flagTemp = false;
                if index == vehicleid
                    continue;
                end
                vehicleTemp = obj.VehicleSet{1, index};
                xTemp = vehicleTemp.GlobalState(1);
                yTemp = vehicleTemp.GlobalState(2);
                yawTemp = vehicleTemp.Yaw;
                positionTemp = [[xTemp, yTemp] + l * [cos(yawTemp), sin(yawTemp)] + w * [cos(yawTemp - pi / 2), sin(yawTemp - pi / 2)];...
                    [xTemp, yTemp] + l * [cos(yawTemp), sin(yawTemp)] + w * [cos(yawTemp + pi / 2), sin(yawTemp + pi / 2)];...
                    [xTemp, yTemp] + l * [cos(yawTemp - pi), sin(yawTemp - pi)] + w * [cos(yawTemp + pi / 2), sin(yawTemp + pi / 2)];...
                    [xTemp, yTemp] + l * [cos(yawTemp - pi), sin(yawTemp - pi)] + w * [cos(yawTemp - pi / 2), sin(yawTemp - pi / 2)]];
                
                for baseIndex = 1:4
                    vectorTemp = positionTemp - basePosition(baseIndex, :);
                    projectionTemp = vectorTemp * vector(baseIndex, :)';
                    flagTemp = flagTemp || (max(projectionTemp) < 0);   % projection is a 1-D vector
                    %   flagTemp = flagTemp || (max(max(projectionTemp)) < 0);
                end
                
                flag = flag && flagTemp;
                
            end            
        end
        
        function obj = car_state_to_global(obj)
            %   TODO: whether obj is changed by method function
            %   function to transform frenet state to global state
            for i = 1:numel(obj.VehicleSet)
                obj.VehicleSet{1, i}.GlobalState = frenet2global(obj.refPath,...
                    obj.VehicleSet{1, i}.FrenetState);
            end
        end
        
        function globalTrajectory = frenet_to_global_trajectory(obj, frenettrajectory)
            %   function to transform frenet state(from planner module) to global state
            %   param: frenettrajectory, n * 6 [s, s_dot, s_ddot, l, l_dos, l_ddos]
            %   output: globalTrajectory, n * 4 [x, y, yaw, v]
            globalTrajectory = zeros(size(frenettrajectory, 1), 4);
            trajectory = frenet2global(obj.refPath, frenettrajectory);
            globalTrajectory(:, 1:2) = trajectory(:, 1:2);
            xDiff = diff(globalTrajectory(:, 1));
            yDiff = diff(globalTrajectory(:, 2));
            xDiff(end + 1) = xDiff(end);
            yDiff(end + 1) = yDiff(end);
            %   TODO: check whether nan will happen
            globalTrajectory(:, 3) = atan(yDiff./xDiff);
            globalTrajectory(:, 4) = xDiff / obj.Conf.TimeResolution;
            index = find(isnan(globalTrajectory));
            globalTrajectory(index) = zeros(size(index));
        end     
        
        function frenetTrajectory = global_to_frenet_trajectory(obj, globalposition)
            %   function to transform global coordinate into frenet coordinate
            %   param: globalposition, global coordinate, n * 2 [x, y]
            %   output: frenetTrajectory, n * 2 [s, l]
            frenetTrajectory = zeros(size(globalposition));
            globalPositionTemp = [globalposition, zeros(size(globalposition, 1), 4)];
            trajectory = global2frenet(obj.refPath, globalPositionTemp);
            frenetTrajectory(:, 1) = trajectory(:, 1);
            frenetTrajectory(:, 2) = trajectory(:, 4);
        end        
        
        function [obj, result] = move_along_trajectory(obj, globaltrajectory, currenttimeindex)
            % TODO: 在每次走完轨迹之后，要记得更新EgoCar的Frenet状态，[s, s_dot, s_ddot, l, l_dos, l_ddos]
            % TODO: 终止状态该如何设置？
            % function to move along global trajectory
            moveTime = size(globaltrajectory, 1);
            for i = 1: moveTime
                timeIndex = currenttimeindex + i;
                obj.CurrentTimeIndex = timeIndex;
                % take one step at globalTrajectory
                % egoCar move
                obj.VehicleSet{1, 1}.GlobalState(1:2) = globaltrajectory(i, 1:2);
                obj.VehicleSet{1, 1}.Yaw = globaltrajectory(i, 3);
                obj.VehicleSet{1, 1}.FrenetState = global2frenet(obj.refPath,...
                    obj.VehicleSet{1, 1}.GlobalState);
                obj.VehicleSet{1, 1}.FrenetState(2) = globaltrajectory(i, 4);
                
                % finish this lane
                if obj.VehicleSet{1, 1}.FrenetState(1) >= obj.Conf.MaxS
                    result.flag  = true;
                    result.finish = true;
                    result.runTimeIndex = i;
                    result.endTimeIndex = timeIndex;
                    break;
                end
                
                % envCar move
                for carIndex = 2: numel(obj.VehicleSet)
                    trajectoryIndex = obj.StaticMap(carIndex);
                    if timeIndex > size(obj.VehicleTrajectory{trajectoryIndex}, 1)
                        continue;
                    elseif timeIndex == size(obj.VehicleTrajectory{trajectoryIndex}, 1)
                        obj.VehicleSet{1, carIndex}.GlobalState = [-10, 0, inf, inf, inf, inf];
                        obj.VehicleSet{1, carIndex}.Yaw = 0;
                        obj.VehicleSet{1, carIndex}.FrenetState = [0, inf, inf, 5.4, inf, inf];
                    else
                        obj.VehicleSet{1, carIndex}.GlobalState(1:2) =...
                            obj.VehicleTrajectory{trajectoryIndex}(timeIndex, 1:2);
                        obj.VehicleSet{1, carIndex}.Yaw =...
                            obj.VehicleTrajectory{trajectoryIndex}(timeIndex, 3);
                        obj.VehicleSet{1, carIndex}.FrenetState =...
                            global2frenet(obj.refPath, obj.VehicleSet{1, carIndex}.GlobalState);
                        obj.VehicleSet{1, carIndex}.FrenetState(2) =...
                            obj.VehicleTrajectory{trajectoryIndex}(timeIndex, 4);
                    end
                end
                % check collision
                if ~obj.check_collision(1)
                    result.flag = false;
                    result.finish = false;
                    result.runTimeIndex = i;
                    result.endTimeIndex = timeIndex;
                    break;
                end
                
            end

            result.flag = true;
            result.finish = false;
            result.runTimeIndex = moveTime;
            result.endTimeIndex = moveTime + currenttimeindex;
        end

        function flag = move_along_trajectory_vistor(obj, globaltrajectory, currenttime)
            moveTime = size(globaltrajectory, 1);
            obj.VistorInstance.Scenario.SampleTime = obj.Conf.TimeResolution;
            plot(obj.VistorInstance.Scenario);
%             chasePlot(obj.VistorInstance.Scenario.Actors(1),'ViewLocation',-[3.6*5, 0],'ViewHeight',10,'ViewPitch',20);
            advance(obj.VistorInstance.Scenario);
            for i = 1: moveTime
                % take one step at globalTrajectory
                % egoCar move
                pause(obj.VistorInstance.Scenario.SampleTime);
                
                obj.VehicleSet{1, 1}.GlobalState(1:2) = globaltrajectory(i, 1:2);
                obj.VehicleSet{1, 1}.Yaw = globaltrajectory(i, 3);
                obj.VehicleSet{1, 1}.FrenetState = global2frenet(obj.refPath,...
                    obj.VehicleSet{1, 1}.GlobalState);
                obj.VehicleSet{1, 1}.FrenetState(2) = globaltrajectory(i, 4);
                
                obj.VistorInstance.Scenario.Actors(1).Position(1:2) = globaltrajectory(i, 1:2);
                obj.VistorInstance.Scenario.Actors(1).Yaw = globaltrajectory(i, 3);
                
                if obj.VehicleSet{1, 1}.FrenetState(1, 1) >= obj.Conf.MaxS
                    break;
                end
                % envCar move
                timeIndex = currenttime + i;
                for carIndex = 2: numel(obj.VehicleSet)
                    trajectoryIndex = obj.StaticMap(carIndex);
                    if timeIndex > size(obj.VehicleTrajectory{trajectoryIndex}, 1)
                        continue;
                    elseif timeIndex == size(obj.VehicleTrajectory{trajectoryIndex}, 1)
                        obj.VehicleSet{1, carIndex}.GlobalState = [-10, 0, inf, inf, inf, inf];
                        obj.VehicleSet{1, carIndex}.Yaw = 0;
                        obj.VehicleSet{1, carIndex}.FrenetState = [0, inf, inf, 5.4, inf, inf];
                        
                        obj.VistorInstance.Scenario.Actors(carIndex).Position(1:2) = [-10, 0];
                        obj.VistorInstance.Scenario.Actors(carIndex).Yaw = 0;
                        
                    else
                        obj.VehicleSet{1, carIndex}.GlobalState(1:2) =...
                            obj.VehicleTrajectory{trajectoryIndex}(timeIndex, 1:2);
                        obj.VehicleSet{1, carIndex}.Yaw =...
                            obj.VehicleTrajectory{trajectoryIndex}(timeIndex, 3);
                        obj.VehicleSet{1, carIndex}.FrenetState =...
                            global2frenet(obj.refPath, obj.VehicleSet{1, carIndex}.GlobalState);
                        obj.VehicleSet{1, carIndex}.FrenetState(2) =...
                            obj.VehicleTrajectory{trajectoryIndex}(timeIndex, 4);
                        
                        obj.VistorInstance.Scenario.Actors(carIndex).Position(1:2) =...
                            obj.VehicleSet{1, carIndex}.GlobalState(1:2);
                        obj.VistorInstance.Scenario.Actors(carIndex).Yaw =...
                            obj.VehicleSet{1, carIndex}.Yaw;
                    end
                end
                advance(obj.VistorInstance.Scenario);
                tic;
                % check collision
                if ~obj.check_collision(1)
                    break;
                end
            end
        end
        
    end
end

