classdef WaypointTrajectoryLibraryGenerator < TrajectoryLibraryGenerator
    
    properties (SetAccess = private)
        x0
        u0
        xf
        uf
        waypoints
        sortorder
    end   
    
    methods
        function obj = WaypointTrajectoryLibraryGenerator(robot)
            obj = obj@TrajectoryLibraryGenerator(robot);
        end
        
        function obj = setInitialState(obj, x0)
            obj.x0 = x0;
        end
        
        function obj = setInitialInput(obj, u0)
            obj.u0 = u0;
        end
        
        function obj = setFinalState(obj, xf)
            obj.xf = xf;
        end
        
        function obj = setFinalInput(obj, uf)
            obj.uf = uf;
        end
        
        function obj = setSortOrder(obj, sortOrder)
            obj.sortorder = sortOrder;
        end
        
        function obj = addWaypoint(obj, qCyclic)
            if numel(qCyclic) ~= numel(obj.cyclicIdx)
                error('Drake:WaypointTrajectoryLibraryGenerator:BadDimensions', 'Waypoints must be specified in the non-cyclic coordinates of the system');
            end
            obj.waypoints{end+1} = qCyclic;
        end
        
        function trajLib = generateTrajectories(obj)
            [xtraj, utraj, info] = obj.solveTrajectoryOptimization();
            segments = obj.getTrajectoryKnots();
            numSegments = numel(segments);
            xtrajs = cell(1, numSegments);
            utrajs = cell(1, numSegments);
            numStates = obj.robot.getNumStates();
            tmp = zeros(numStates, 1);
            for i = 1:numSegments
                segmentKnots = segments{i};
                segmentBreaks = xtraj.pp.breaks(segmentKnots(1):segmentKnots(2));
                trajSegment = xtraj.eval(segmentBreaks);
                tmp(obj.cyclicIdx) = trajSegment(obj.cyclicIdx, 1);
                xtrajpp = PPTrajectory(spline(segmentBreaks - segmentBreaks(1), trajSegment - repmat(tmp, 1, numStates)));
                utrajpp = PPTrajectory(spline(segmentBreaks - segmentBreaks(1), utraj.eval(segmentBreaks)));
                
                [xtrajResampled, utrajResampled] = obj.resampleTrajectory(xtrajpp, utrajpp, 50); %TODO: make this configurable
                xtrajs{obj.sortorder(i)} = xtrajResampled.setOutputFrame(obj.robot.getStateFrame);
                utrajs{obj.sortorder(i)} = utrajResampled.setOutputFrame(obj.robot.getInputFrame);
            end
            trajLib = TrajectoryLibrary(xtrajs, utrajs);
        end
        
    end
    
    methods (Access = private)
        function [xtraj, utraj] = resampleTrajectory(obj, xtraj, utraj, knots)
            minimum_duration = .1;
            maximum_duration = xtraj.pp.breaks(end);
            
            prog = DircolTrajectoryOptimization(obj.robot, knots, [minimum_duration maximum_duration]);
            
            x0 = xtraj.eval(xtraj.pp.breaks(1));
            u0 = utraj.eval(utraj.pp.breaks(1));
            
            xf = xtraj.eval(xtraj.pp.breaks(end));
            uf = utraj.eval(utraj.pp.breaks(end));
            
            prog = prog.addStateConstraint(ConstantConstraint(x0), 1);
            prog = prog.addInputConstraint(ConstantConstraint(u0), 1);
            
            prog = prog.addStateConstraint(ConstantConstraint(xf), knots);
            prog = prog.addInputConstraint(ConstantConstraint(uf), knots);
            
            prog = prog.addRunningCost(@(dt, x, u) obj.cost(dt, x, u));
            prog = prog.addFinalCost(@(t, x) obj.finalCost(t, x));
            
            traj_init.x = PPTrajectory(foh([0, maximum_duration],[x0, xf]));
            traj_init.u = ConstantTrajectory(u0);
            
            [xtraj,utraj,~,~,~] = prog.solveTraj(maximum_duration,traj_init);
            
        end
        
        function [xtraj, utraj, info] = solveTrajectoryOptimization(obj)
            N = (obj.trajLength + 1) * (numel(obj.waypoints) + 2) - 1;
                        
            %todo: make these configurable
            minimum_duration = .1;
            maximum_duration = 10;
            
            prog = DircolTrajectoryOptimization(obj.robot, N, [minimum_duration maximum_duration]); 
            
            %initial conditions
            prog = prog.addStateConstraint(ConstantConstraint(obj.x0), 1);
            prog = prog.addInputConstraint(ConstantConstraint(obj.u0), 1);
            
            %final conditions
            prog = prog.addStateConstraint(ConstantConstraint(obj.xf), N);
            prog = prog.addInputConstraint(ConstantConstraint(obj.uf), N);
            
            %waypoint constraints
            prog = obj.addWaypointConstraints(prog);
            prog = obj.addSequentialCompositionConstraints(prog);
            
            prog = prog.addRunningCost(@(dt, x, u) obj.cost(dt, x, u));
            prog = prog.addFinalCost(@(t, x) obj.finalCost(t, x));
            
            
            tf0 = 10;                      
            traj_init.x = PPTrajectory(foh([0, tf0],[obj.x0, obj.xf]));
            traj_init.u = ConstantTrajectory(obj.u0);
            
            [xtraj,utraj,~,~,info] = prog.solveTraj(tf0,traj_init);
        end
        
        function prog = addWaypointConstraints(obj, prog)
            numStates = obj.robot.getNumStates();
            for i = 1:numel(obj.waypoints)
                xm_lb = -inf(numStates, 1);
                xm_ub  = inf(numStates, 1);
                xm_lb(obj.cyclicIdx) = obj.waypoints{i};
                xm_ub(obj.cyclicIdx) = obj.waypoints{i};
                prog = prog.addStateConstraint(BoundingBoxConstraint(xm_lb, xm_ub), 1 + obj.trajLength * i);
            end

        end
        
        function prog = addSequentialCompositionConstraints(obj, prog)
            
            numConstraints = numel(obj.nonCyclicIdx);
            numStates = obj.robot.getNumStates();
            
            A = zeros(numConstraints, 2 * numStates);
            for i = 1:numConstraints
                A(i, obj.nonCyclicIdx(i)) = 1;
                A(i, obj.nonCyclicIdx(i) + numStates) = -1;
            end
            prog = prog.addStateConstraint(LinearConstraint(zeros(numConstraints,1), zeros(numConstraints,1), A), obj.getTrajectoryKnots);
        end
        
        function knots = getTrajectoryKnots(obj)
            numTrajSegments = numel(obj.waypoints) - 1;
            knots = cell(1, numTrajSegments);
            for i = 1:numTrajSegments
                knots{i} = [1 + i * obj.trajLength, 1 + (i + 1) * obj.trajLength];
            end
        end
        
        function [g,dg] = cost(obj, dt,x,u)
            R = eye(4);
            g = u'*R*u;
            dg = [zeros(1,1+size(x,1)),2*u'*R];
        end

        function [h,dh] = finalCost(obj, t,x)
            h = t;
            dh = [1,zeros(1,size(x,1))];
        end
        
    end
    
end

