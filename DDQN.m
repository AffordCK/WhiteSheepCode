classdef DDQN
    %DDQN the DDQN class
    %    
    
    properties
        ObsInfo                 % the observation object
        ActInfo                 % the action object
        EnvInfo                 % the rl env object
        Agent                   % the training agent
        EnvObject               % the env controlled object
    end
    
    methods
        function obj = DDQN()
            %   Initiate the object of observation
            obj.ObsInfo = rlNumericSpec([14, 1]);
            obj.ObsInfo.Name = "observations";
            obj.ObsInfo.LowerLimit = [0, 0,...
                                  0, -inf,...
                                  0, -inf,...
                                  0, -inf,...
                                  -200, -inf,...
                                  -200, -inf,...
                                  -200, -inf];
            obj.ObsInfo.UpperLimit = [500, inf,...
                                  200, inf,...
                                  200, inf,...
                                  200, inf,...
                                  0, inf,...
                                  0, inf,...
                                  0, inf];
                              
            %   Initiate the object of action
            obj.ActInfo = rlNumericSpec([7, 1]);
            obj.ActInfo.Name = "actions";
            obj.ActInfo.LowerLimit = [0, -inf, -inf, -inf, -inf, -inf, 0];
            obj.ActInfo.UpperLimit = [inf, inf, inf, inf, inf, inf, inf];
            
            %   Initiate the rl env object
            obj.EnvInfo = rlFunctionEnv(obj.ObsInfo, obj.ActInfo, obj.step_fcn, obj.reset_fcn);
            
            %   Initiate the agent object
            dnn = [
                %   https://ww2.mathworks.cn/help/deeplearning/ref/nnet.cnn.layer.featureinputlayer.html
                featureInputLayer(obj.ObsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'State'),...          
                fullyConnectedLayer(100, 'Name', 'CriticStateFC1'),...
                reluLayer('Name', 'CriticRelu1'),...
                fullyConnectedLayer(50, 'Name', 'CriticStateFC2'),...
                reluLayer('Name', 'CriticCommonRelu'),...
                fullyConnectedLayer(length(obj.ActInfo.Elements),'Name','output')];        
            
            %   https://ww2.mathworks.cn/help/reinforcement-learning/ref/rlrepresentationoptions.html
            criticOpts = rlRepresentationOptions('LearnRate', 0.001, 'GradientThreshold', 1);
            critic = rlQValueRepresentation(dnn, obj.ObsInfo, obj.ActInfo, 'Observation',{'state'},criticOpts);
            agentOpts = rlDQNAgentOptions(...
                'UseDoubleDQN', true, ...    
                'TargetSmoothFactor', 1, ...
                'TargetUpdateFrequency', 4, ...   
                'ExperienceBufferLength', 100000, ...
                'DiscountFactor', 0.99, ...
                'MiniBatchSize', 256);
            obj.Agent = rlDQNAgent(critic, agentOpts);
            
            %   Initiate the object of env
            obj.EnvObject = Env();
            [obj.EnvObject, vehicleSet, vehiclesTrajectory] = obj.EnvObject.load_scenario("Scenario/01.mat", 1);
            
            %   Load road object
            x = [-10, 0: 100: 500];
            y = zeros(size(x));
            waypoint = [x', y'];
            obj.EnvObject = obj.EnvObject.load_road(waypoint, vehicleSet, vehiclesTrajectory);
        end
        
        
        
        %   https://ww2.mathworks.cn/help/reinforcement-learning/ug/create-custom-reinforcement-learning-environment-in-matlab.html
        %   build the step function
        function [Observation, Reward, IsDone, LoggedSignals] = step_fcn(Action, LoggedSignals)
            
        end
        
        %   build the reset function
        function [InitialObservation, LoggedSignals] = reset_fcn()
            
        end
        
        %   validate rl env
        function validate_fcn(obj)
            validateEnvironment(obj.EnvInfo);
        end
        
        function train(obj)
            trainOpts = rlTrainingOptions(...
                'MaxEpisodes', 1000, ...
                'MaxStepsPerEpisode', 500, ...
                'Verbose', false, ...
                'Plots', 'training-progress',...
                'StopTrainingCriteria', 'AverageReward',...
                'StopTrainingValue', 480);
            trainingStats = train(obj.Agent, obj.EnvInfo, trainOpts);
        end
        
    end
end

