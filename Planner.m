classdef Planner
    %   PLANNER class
    %   used to plan in frenet coordinate
    
    properties
        QuinticCoeS
        QuinticCoeD
        QuarticCoeD
    end
    
    methods
        function obj = Planner()
            %   PLANNER construct function
            %   nothing, hia hia
            fprintf("[%s]: Creating Planner object...\n", datestr(now));
        end
        
        function result = get_trajectory(obj, startstate, endstate, T, timeresolution, flag)
            %   function to get planning trajectory [s, s_dot, s_ddot, l, l_dos, l_ddos]
            %   param: startstate, the initial state, [s0, s0_dot, s0_ddot, l0, l0_dos, l0_ddos]
            %   param: endstate, the final state, [sf, sf_dot, sf_ddot, lf, lf_dos, lf_ddos]
            %   param: T, planning time
            %   param: timeresolution, Temporal resolution
            %   param: flag, true with quintic_planning_l, otherwise
            %   quartic_planning_l
            resultS = quintic_planning_s(obj, startstate(1:3), endstate(1:3), T, timeresolution);
            if flag
                resultL = quintic_planning_l(obj, startstate(4:6), endstate(4:6), resultS);
            else
                resultL = quartic_planning_l(obj, startstate(4:6), endstate(4:6), resultS);
            end
            result.t = resultS.t;
            result.trajectory = [resultS.s, resultS.s_dot, resultS.s_ddot, resultL.l,...
                resultL.l_dos, resultL.l_ddos];
        end
        
        function flag = check_trajectory(obj, trajectory)
            %   function to check whether the trajectory(generated from get_trejectory function)
            %       is feasible
            %   param: result, the trajectory generated from get_trejectory function
            %   output: flag, true when the trajectory is feasible,
            %       otherwise false
            flag = all(trajectory(:, 2) > 0);
        end
        
        function result = quintic_planning_s(obj, startpoint, endpoint, T, timeresolution)
            %   used to quintic planning in s direction
            %   param: startpoint, [s0, s0_dot, s0_ddot]
            %   param: endpoint, [sf, sf_dot, sf_ddot]
            %   param: T, planning time
            if numel(startpoint) ~= numel(endpoint) && numel(startpoint) ~= 3
                error('invaild input');
            end
            t = [0: timeresolution: T]';
            a0 = startpoint(1);
            a1 = startpoint(2);
            a2 = startpoint(3) / 2;
      
            A = [T^3,       T^4,        T^5;...
                 3 * T^2,   4 * T^3,    5 * T^4;...
                 6 * T,     12 * T^2,   20 * T^3];
            B = [endpoint(1) - a0 - a1 * T - a2 * T^2;...
                 endpoint(2) - a1 - 2 * a2 * T;...
                 endpoint(3) - 2 * a2];
            coeff = inv(A) * B;
            
            a3 = coeff(1);
            a4 = coeff(2);
            a5 = coeff(3);
            obj.QuinticCoeS = [a0, a1, a2, a3, a4, a5];
            result.t = t;
            result.s = a0 + a1 * t + a2 * t.^2 + a3 * t.^3 + a4 * t.^4 + a5 * t.^5;
            result.s_dot = a1 + 2 * a2 * t + 3 * a3 * t.^2 + 4 * a4 * t.^3 + 5 * a5 * t.^4;
            result.s_ddot = 2 * a2 + 6 * a3 * t + 12 * a4 * t.^2 + 20 * a5 * t.^3;
        end
        
        function result = quintic_planning_l(obj, startpoint, endpoint, resultS)
            %   used to quintic planning in l direction
            %   param: startpoint, [l0, l0_dos, l0_ddos]
            %   param: endpoint, [lf, lf_dos, lf_ddos]
            %   param: resultS, planning S from quintic_planning_s
            if numel(startpoint) ~= numel(endpoint) && numel(startpoint) ~= 3
                error('invaild input');
            end
            s = resultS.s - resultS.s(1);
            sEnd = s(end);
            b0 = startpoint(1);
            b1 = startpoint(2);
            b2 = startpoint(3) / 2;
            
            A = [sEnd^3,       sEnd^4,        sEnd^5;...
                 3 * sEnd^2,   4 * sEnd^3,    5 * sEnd^4;...
                 6 * sEnd,     12 * sEnd^2,   20 * sEnd^3];
            B = [endpoint(1) - b0 - b1 * sEnd - b2 * sEnd^2;...
                 endpoint(2) - b1 - 2 * b2 * sEnd;...
                 endpoint(3) - 2 * b2];
            coeff = inv(A) * B;
            
            b3 = coeff(1);
            b4 = coeff(2);
            b5 = coeff(3);
            obj.QuinticCoeD = [b0, b1, b2, b3, b4, b5];
            
            result.t = resultS.t;
            result.s = resultS.s;
            result.l = b0 + b1 * s + b2 * s.^2 + b3 * s.^3 + b4 * s.^4 + b5 * s.^5;
            result.l_dos = b1 + 2 * b2 * s + 3 * b3 * s.^2 + 4 * b4 * s.^3 + 5 * b5 * s.^4;
            result.l_ddos = 2 * b2 + 6 * b3 * s + 12 * b4 * s.^2 + 20 * b5 * s.^3;
            result.l_dot = result.l_dos .* resultS.s_dot;
            result.l_ddot = result.l_ddos .* (resultS.s_dot.^2) + result.l_dos .* (resultS.s_ddot);
        end

        function result = quartic_planning_l(obj, startpoint, endpoint, resultS)
            %   used to quartic planning in l direction
            %   param: startpoint, [l0, l0_dos, l0_ddos]
            %   param: endpoint, [lf, lf_dos, lf_ddos]
            %   param: resultS, planning S from quintic_planning_s
            s = resultS.s - resultS.s(1);
            sEnd = s(end);
            b0 = startpoint(1);
            b1 = startpoint(2);
            b2 = startpoint(3) / 2;
            
            A = [sEnd^3,       sEnd^4;...
                 3 * sEnd^2,   4 * sEnd^3];
            B = [endpoint(1) - b0 - b1 * sEnd - b2 * sEnd^2;...
                 endpoint(2) - b1 - 2 * b2 * sEnd];
            coeff = inv(A) * B;
            
            b3 = coeff(1);
            b4 = coeff(2);
            obj.QuarticCoeD = [b0, b1, b2, b3, b4];
            
            result.t = resultS.t;
            result.s = resultS.s;
            result.l = b0 + b1 * s + b2 * s.^2 + b3 * s.^3 + b4 * s.^4;
            result.l_dos = b1 + 2 * b2 * s + 3 * b3 * s.^2 + 4 * b4 * s.^3;
            result.l_ddos = 2 * b2 + 6 * b3 * s + 12 * b4 * s.^2;
            result.l_dot = result.l_dos .* resultS.s_dot;
            result.l_ddot = result.l_ddos .* (resultS.s_dot.^2) + result.l_dos .* (resultS.s_ddot);
        end
        
    end
end

