classdef Logger
    %LOGGER Class
    %   used to logger running info
    
    properties
        LogFileName                 % the file name of log
        
    end
    
    methods
        function obj = Logger(varargin)
            %   Logger construction function
            param = inputParser;
            addParameter(param, 'logfilename', 'LogFile');
            param.parse(varargin{:});
            
            obj.LogFileName = param.Results.logfilename;
            %   start log when implement construction function
            diary(obj.LogFileName);
        end
        
        function start_log(obj)
            %   function to start log after stop log
            diary(obj.LogFileName);
        end
        
        function stop_log(obj)
            %   function to stop log
            diary off;
        end
        
        function flag = check_log(obj)
            %   check whether the log is on
            flag = strcmp(get(0, 'Diary'), 'on');
        end
        
        function show_log(obj)
            %   show the context on the log file
            if ~exist(obj.LogFileName, 'file')
                error('no log file in this directory');
            end
            type(obj.LogFileName);
        end
        
        function clear_log(obj)
            %   clear the context on the log file
            fid = fopen(obj.LogFileName, 'w');
            fclose(fid);
        end
        
    end
end

