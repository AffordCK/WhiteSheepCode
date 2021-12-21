classdef Reward
    %   Rrward Class
    %   Define reward score
    %   All reward come on Frenet coordinate
    %   TODO: modify each reward
    
    properties
        Conf                                        % Configuration info
        
        InFeasibleReward                            % the reward of infeasible trajectory   
        
        CollisionReward                             % the reward of collision
        
        DistanceRewardGenerator                     % the generator of distance reward
        DistanceWeight                              % the weight of distance reward
        DistanceReward                              % the reward of driving distance
        
        TimeRewardGenerator                         % the generator of distance time
        TimeWeight                                  % the weight of driving time
        TimeReward                                  % the reward of driving time
        
        LaneOffsetRewardGenerator                   % the generator of lane offset reward
        LaneOffsetReward                            % the reward of lane offset
        LaneOffsetWeight                            % the weight of lane offset reward
        
        VelocityReward                              % the reward of driving velocity
        VelocityWeight                              % the weight of driving velocity reward, negative
        
        LongJerkReward                              % the reward of longitudinal jerk
        LongJerkWeight                              % the weight of longitudinal jerk, negative
        
        LatJerkReward                               % the reward of lateral jerk
        LatJerkWeight                               % the weight of lateral jerk, negatitive
        
        TotalReward                                 % the total reward
    end
    
    methods
        function obj = Reward()
            fprintf("[%s]: Creating Reward object...\n", datestr(now));
            
            obj.Conf = conf();
            
            obj.InFeasibleReward = obj.Conf.InFeasibleReward;  

            obj.CollisionReward = obj.Conf.CollisionReward;   

            obj.DistanceRewardGenerator = makedist('Normal', 'mu', obj.Conf.DistanceMean,...
                'sigma', obj.Conf.DistanceVariance);        
            obj.DistanceWeight = obj.Conf.DistanceWeight;       

            obj.TimeRewardGenerator = makedist('Normal', 'mu', obj.Conf.TimeMean,...
                'sigma', obj.Conf.TimeVariance);       
            obj.TimeWeight = obj.Conf.TimeWeight;              

            obj.LaneOffsetRewardGenerator = {};
            for i = 1: obj.Conf.LaneNum
                obj.LaneOffsetRewardGenerator{i} = makedist('Normal',...
                    'mu', obj.Conf.LaneOffsetMean(1, i),...
                    'sigma', obj.Conf.LaneOffsetVariance(1, i));
            end
                     
            obj.LaneOffsetWeight = obj.Conf.LaneOffsetWeight;   
                     
            obj.VelocityWeight = obj.Conf.VelocityWeight;        
                      
            obj.LongJerkWeight = obj.Conf.LongJerkWeight;    
                         
            obj.LatJerkWeight = obj.Conf.LatJerkWeight;      

            obj.TotalReward = 0;
        end
        
        function obj = cal_distance_reward(obj, distance)
            %   calculate the reward of distance
            %   param: distance, the longitudinal distance in trajectory
            obj.DistanceReward = pdf(obj.DistanceRewardGenerator, distance);
        end

        function obj = cal_time_reward(obj, time)
            %   calculate the reward of traveling time
            %   param: time, the driving time in trajectory
            obj.TimeReward = pdf(obj.TimeRewardGenerator, time);
        end
        
        function obj = cal_offset_reward(obj, offset)
            %   calculate the reward of lateral offset
            %   param: offset, the final lateral offset in the lane
            reward = 0;
            for i = 1: numel(obj.LaneOffsetRewardGenerator)
                reward = reward + pdf(obj.LaneOffsetRewardGenerator{i}, offset);
            end
            obj.LaneOffsetReward = reward;
        end
        
        function obj = cal_velocity_reward(obj, frenettrajectory)
            %   calculate the reward of velocity
            %   param: frenettrajectory, the driving trajectory
            obj.VelocityReward = sum(abs(frenettrajectory(:, 2) - obj.Conf.MaxSpeed)) * obj.Conf.TimeResolution;
        end
        
        function obj = cal_longjerk_reward(obj, frenettrajectory)
            %   calculate the reward of longitudinal jerk
            %   param: frenettrajectory, the driving trajectory
            aAcceleration = diff(frenettrajectory(:, 3));
            aAcceleration(end + 1) = aAcceleration(end);
            aAcceleration = aAcceleration / obj.Conf.TimeResolution;
            obj.LongJerkReward = sum(abs(aAcceleration)) * obj.Conf.TimeResolution;
        end
        
        function obj = cal_latjerk_reward(obj, frenettrajectory)
            %   calculate the reward of lateral jerk
            %   param: frenettrajectory, the driving trajectory
            aAcceleration = diff(frenettrajectory(:, 6));
            aAcceleration(end + 1) = aAcceleration(end);
            sDiff = diff(frenettrajectory(:, 1));
            sDiff(end + 1) = sDiff(end);
            aAcceleration = aAcceleration ./ sDiff;
            obj.LatJerkReward = sum(abs(aAcceleration)) * obj.Conf.TimeResolution;
        end
        
        function obj = cal_all_reward(obj, frenettrajectory)
            %   calculate all reward aforementioned
            %   param: frenettrajectory, the driving trajectory
            obj = obj.cal_distance_reward(frenettrajectory(end, 1) - frenettrajectory(1, 1));
            obj = obj.cal_time_reward(size(frenettrajectory, 1) * obj.Conf.TimeResolution);
            obj = obj.cal_offset_reward(frenettrajectory(end, 4));
            obj = obj.cal_velocity_reward(frenettrajectory);
            obj = obj.cal_longjerk_reward(frenettrajectory);
            obj = obj.cal_latjerk_reward(frenettrajectory);
        end
        
        function obj = cal_total_reward(obj)
            %   calculate total reward
            obj.TotalReward = obj.DistanceReward * obj.DistanceWeight +...
                obj.TimeReward * obj.TimeWeight +...
                obj.LaneOffsetReward * obj.LaneOffsetWeight +...
                obj.VelocityReward * obj.VelocityWeight +...
                obj.LongJerkReward * obj.LongJerkWeight +...
                obj.LatJerkReward * obj.LatJerkWeight;
            fprintf("TotalReward is: %d\n", obj.TotalReward);
            fprintf("Distance Reward is: %d\n", obj.DistanceReward);
            fprintf("Time Reward is: %d\n", obj.TimeReward);
            fprintf("Lane Offset Reward is %d\n", obj.LaneOffsetReward);
            fprintf("Long Jerk Reward is: %d\n", obj.LongJerkReward);
            fprintf("Lateral Jerk Reward is: %d\n", obj.LatJerkReward);
        end
        
    end
end

