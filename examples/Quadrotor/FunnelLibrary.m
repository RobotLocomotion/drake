classdef FunnelLibrary
    properties(SetAccess = private)
        trajLib
        funnels
    end
    
    properties(SetAccess = public)
        options;
    end
    
    methods
        function obj = FunnelLibrary(trajectoryLibrary)
            obj.trajLib = trajectoryLibrary;
            numTrajs = numel(trajectoryLibrary.trajectories);
            obj.funnels = cell(1, numTrajs);
            obj.options = struct();
            obj.options.saturations = false;
            obj.options.rho0 = 1;
            obj.options.degL1 = 2;
            obj.options.max_iterations = 1;
            obj.options.backoff_percent = 5;
        end
        
        
        function obj = computeFunnels(obj, robot, scaleFactor, funnelLength)
            numTrajs = numel(obj.trajLib.trajectories);
            for i = 1:numTrajs
                xtraj = obj.trajLib.trajectories{i}.xtraj;
                utraj = obj.trajLib.trajectories{i}.utraj;
                Vtv = obj.trajLib.trajectories{i}.Vtv;
                tv = obj.trajLib.trajectories{i}.tv;
                V = Vtv*scaleFactor;
                
                sysCl = feedback(robot, tv);
                
                sysClPoly = taylorApprox(sysCl, xtraj, [], 3);
                sysPoly = taylorApprox(robot, xtraj, utraj, 3);
                
                ts = Vtv.S.getBreaks();
                ts = linspace(ts(1),ts(end),funnelLength);
                ts = ts(1:end-1);
                
                G0 = V.S.eval(0) / obj.options.rho0 * 1.01;
                

                [V,rho,Phi]=sampledFiniteTimeReach_B0(sysClPoly,sysPoly,V,G0,0,tv,ts,xtraj,utraj, obj.options);
                
                obj.funnels{i} = struct();
                obj.funnels{i}.V = V;
                obj.funnels{i}.rho = rho;
                obj.funnels{i}.Phi = Phi;
            end
        end
    end
    
end

