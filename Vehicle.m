classdef Vehicle
    %   Vehicle Class
    %   Vehicle consist of position, dynamic property, and kinematic
    %   property ...
    %   TODO: car coordinate, global coordinate and frenet coordinate  
    properties
%         Id                              % car id
        Length                                  % car length
        Width                                   % car width
        GlobalState = zeros(1, 6)               % [x, y, theta, kappa, kappa_dot, s]
%         CartesianState                        % [x, y, theta, kappa, speed, acceleration]
        FrenetState = zeros(1, 6)               % [s, s_dot, s_ddot, l, l_dos, ll_dos];
        FrenetVelocity = zeros(1, 2)            % FrenetVelocity = [s_dot, l_dot], velocity on frenet frame
        
        Yaw                                     % the yaw of the
        LongVelocity                            % longitudinal velocity, velocity on car frame
        LatVelocity                             % lateral velocity, velocity on car frame
        
        VelocityX                               % velocity on global frame x axis
        VelocityY                               % velocity on global frame y axis
        GlobalVelocity = zeros(1, 2)            % GlobalVelocity = [VelocityX, VelocityY]
        
        Trajectory                              % Global Trajectory
        
        Lane                                    % current lane id
        Conf                                    % configuration info
    end
    
    methods
        function obj = Vehicle(frenetposition, varargin)
            %   Vehicle construction function
            %   param: frenetposition [s, l] in frenet coordinate
            %   param: longvelocity longitudinal velocity
            param = inputParser;
            addParameter(param, 'longvelocity', 0);
            addParameter(param, 'latvelocity', 0);
%             addParameter(param, 'frenetvelocity', [0, 0]);
            addParameter(param, 'length', 3.6);
            addParameter(param, 'width', 1.8);
            addParameter(param, 'yaw', 0);
            param.parse(varargin{:});
            
            obj.Conf = conf();
            
            obj.FrenetState(1) = frenetposition(1);
            obj.FrenetState(4) = frenetposition(2);
            obj.Yaw = param.Results.yaw;
            obj.LongVelocity = param.Results.longvelocity;
            obj.FrenetState(2) = obj.LongVelocity;
            obj.LatVelocity = param.Results.latvelocity;
            obj.Length = param.Results.length;
            obj.Width = param.Results.width;
            obj.Lane = ceil(obj.FrenetState(4) / obj.Conf.LaneWidth);
            
        end    
    end
end

