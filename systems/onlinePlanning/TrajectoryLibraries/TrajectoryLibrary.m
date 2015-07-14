classdef TrajectoryLibrary    
    
    properties(SetAccess = public)
        trajectories
        options
    end
    
    methods
        function obj = TrajectoryLibrary(xtrajs, utrajs)
            numTrajs = numel(xtrajs);
            obj.trajectories = cell(1, numTrajs);
            obj.options = struct();
            for i = 1:numTrajs
                obj.trajectories{i} = struct();
                obj.trajectories{i}.xtraj = xtrajs{i};
                obj.trajectories{i}.utraj = utrajs{i};
                obj.trajectories{i}.Vtv = [];
                obj.trajectories{i}.tv = [];
            end
        end
        
        function obj = plotTrajectories(obj, slice)
          numTrajs = numel(obj.trajectories);
          for i = 1:numTrajs
              fnplt(obj.trajectories{i}.xtraj, slice);
              hold on;
          end
        end
        
        function obj = setupFrames(obj, robot)
            numTrajs = numel(obj.trajectories);
            for i = 1:numTrajs
                obj.trajectories{i}.xtraj = obj.trajectories{i}.xtraj.setOutputFrame(robot.getStateFrame());
                obj.trajectories{i}.utraj = obj.trajectories{i}.utraj.setOutputFrame(robot.getInputFrame());
            end
        end
        
        function obj = stabilizeTrajectories(obj, robot, Q, R, Qf)
            numTrajs = numel(obj.trajectories);
            for i = 1:numTrajs
                [tv,Vtv] = tvlqr(robot, obj.trajectories{i}.xtraj, obj.trajectories{i}.utraj, Q, R, Qf, obj.options);
                obj.trajectories{i}.tv = tv;
                obj.trajectories{i}.Vtv = Vtv;
            end
        end
        
        function plot(obj)
            numTrajs = numel(obj.trajectories);
            for i = 1:numTrajs
                fnplt(obj.trajectories{i}.xtraj, [1 2]);
                hold on;
            end
        end
    end
    
end

