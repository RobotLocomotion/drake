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
        
        function obj = plotFunnels(obj, robot)
            numFunnels = numel(obj.funnels);
            for i = 1:numFunnels
                Vxframe = obj.funnels{i}.V.inFrame(robot.getStateFrame());
                xtraj = obj.trajLib.trajectories{i}.xtraj;
                options.plotdims = [1 2];
                options.x0 = xtraj;
                options.ts = obj.funnels{i}.ts;
                options.inclusion = 'projection';
                plotFunnel(Vxframe, options);
                hold on;
                fnplt(xtraj ,[1 2]); 
            end
        end
        
        function obj = computeFunnels(obj, robot, scaleFactor, funnelLength, radius)
            numTrajs = numel(obj.trajLib.trajectories);
            %numTrajs = 1; %DEBUGGING
            for i = 1:numTrajs
                disp('building funnel')
                xtraj = obj.trajLib.trajectories{i}.xtraj;
                utraj = obj.trajLib.trajectories{i}.utraj;
                Vtv = obj.trajLib.trajectories{i}.Vtv;
                tv = obj.trajLib.trajectories{i}.tv;
                V = Vtv*scaleFactor;
                
                sysCl = feedback(robot, tv);
                disp('taylor approximating dynamics')
                sysClPoly = taylorApprox(sysCl, xtraj, [], 3);
                sysPoly = taylorApprox(robot, xtraj, utraj, 3);
                
                ts = Vtv.S.getBreaks();
                ts = linspace(ts(1),ts(end),funnelLength);
                ts = ts(1:end-1);
                
                G0 = V.S.eval(0) / obj.options.rho0 * 1.01;
                
                disp('computing reachable set')
                [V,rho,Phi]=sampledFiniteTimeReach_B0(sysClPoly,sysPoly,V,G0,0,tv,ts,xtraj,utraj, obj.options);
                
                obj.funnels{i} = struct();
                obj.funnels{i}.V = V;
                obj.funnels{i}.rho = rho;
                obj.funnels{i}.Phi = Phi;
                
                C = [eye(3), zeros(3,9)]; %TODO: generalize the projection matrix

                obj.funnels{i}.xtraj = xtraj;
                obj.funnels{i}.utraj = utraj;
                obj.funnels{i}.V = V;
                obj.funnels{i}.controller = tv;
                obj.funnels{i}.ts = ts;

                for j = 1:length(ts)
                    S = V.S.eval(ts(j));
                    s1 = V.s1.eval(ts(j))';
                    x0 = -0.5*(S\s1');
                    Sp = inv(C*(S\C'));
                    obj.funnels{i}.Sp{j} = Sp;
                    obj.funnels{i}.s1p{j} = -2*x0'*C'*Sp;
                    obj.funnels{i}.s2p{j} = (C*x0)'*Sp*(C*x0);
                end

                x0 = xtraj.eval(ts);
                obj.funnels{i}.x0 = x0;    
                
                for j = 1:length(ts)
                    Sp = obj.funnels{i}.Sp{j};
                    [VV,DD] = eig(Sp);
                    l1 = 1/sqrt(DD(1,1)) + radius;
                    l2 = 1/sqrt(DD(2,2)) + radius;
                    l3 = 1/sqrt(DD(3,3)) + radius;
                    d1 = 1/l1^2;
                    d2 = 1/l2^2;
                    d3 = 1/l3^2;
                    D2 = diag([d1,d2,d3]);
                    Sp2 = VV*D2*VV';

                    obj.funnels{i}.cS{j} = chol(Sp2);
                end

                obj.funnels{i}.xyz = x0(1:3, :);
                obj.funnels{i}.S0 = V.S.eval(0);
            end
        end
    end
    
end

