classdef TrajectoryLibraryGenerator
    
    properties (SetAccess = private)
        cyclicIdx
        nonCyclicIdx
        robot
        trajLength
    end
    
    methods (Abstract)
        trajLib = generateTrajectories(obj);
    end
    
    methods
        function obj = TrajectoryLibraryGenerator(robot)
            obj.robot = robot;
        end
        
        function obj = setCyclicCoordinateIndexes(obj, cyclicIdx)
            obj.cyclicIdx = cyclicIdx;
            obj.nonCyclicIdx = setdiff(1:obj.robot.getNumStates(), cyclicIdx);
        end
        
        function obj = setTrajectoryLength(obj, trajLength)
            obj.trajLength = trajLength;
        end
    end
end

