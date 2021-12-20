classdef conf
    %   CONF class
    %   define some default parameter
    
    properties
        LaneNum                         % the num of the lane
        LaneWidth                       % the width of the lane
        LaneLength                      % the length of the road
        MaxSpeed                        % the maxspeed in the road
        MaxS                            % the max s
        
        ObserScope                      % the Scope of Observation of a car
        
        TimeResolution                  % temporal resolution
        
        % all reward should stay on Frenet coordinate
        % Reward in the target
        InFeasibleReward                % the reward of infeasible trajectorys
        CollisionReward                 % the reward of collision
        TimeMean                        % the mean of time
        TimeVariance                    % the variance of time
        TimeWeight                      % the reward weight of time
        
        DistanceMean                    % the mean of distance
        DistanceVariance	         	% the variance of distance
        DistanceWeight                  % the reward weight of distance
        
        LaneOffsetMean                  % the mean of lane offset (for mixed normal distribution)
        LaneOffsetVariance              % the variance of lane offset (for mixed normal distribution)
        LaneOffsetWeight                % the weight of lane offset (for mixed normal distribution)
        
        % Reward in the process
        VelocityWeight                  % the reward weight of velocity, should be negative
        LongJerkWeight                  % the weight of longitudinal jerk, should be negative
        LatJerkWeight                   % the weight of lateral jerk, should be negative
    end
    
    methods
        function obj = conf()
            obj.LaneNum = 3;                    
            obj.LaneWidth = 3.6;                
            obj.LaneLength = 500;  
            obj.MaxSpeed = 35;             
            obj.MaxS = 420;             

            obj.ObserScope = 200;           

            obj.TimeResolution = 0.04;      


            obj.InFeasibleReward = -10;        
            obj.CollisionReward = -10;
            
            obj.TimeMean = 3;                 
            obj.TimeVariance = 1;               
            obj.TimeWeight = 10;               

            obj.DistanceMean = obj.MaxSpeed * obj.TimeMean;                  
            obj.DistanceVariance = 1;             
            obj.DistanceWeight = 10;          

            obj.LaneOffsetMean = ([1: obj.LaneNum] - 1 / 2) * obj.LaneWidth;
                                              
            obj.LaneOffsetVariance = ones(1, obj.LaneNum);
                                           
            obj.LaneOffsetWeight = 10;        


            obj.VelocityWeight = 1;        
            obj.LongJerkWeight = 1;              
            obj.LatJerkWeight = 1;              
        end
    end
end

