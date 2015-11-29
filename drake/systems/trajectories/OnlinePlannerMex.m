classdef OnlinePlannerMex < DrakeSystem
    
    properties
        % Constant properties
        funnelLibrary;
        funnelLibraryReduced;
        t_replan;
        cyclicIdx;
        
        % Mutable properties
        currentFunnel;
        currentObstacles;
        t_start;
        x_start;
        init_replan;
        
    end
    
    methods
        
        % construction
        function obj = OnlinePlannerMex(funnelLibrary,currentObstacles,currentFunnel,cyclicIdx,t_replan,t_start,x_start,num_inputs)
            
            disp('Setting up online planner...');
            
            obj = obj@DrakeSystem(0,0,length(x_start),num_inputs,0,0);
            
            obj.funnelLibrary = funnelLibrary;
            
            obj.currentObstacles = currentObstacles;
            obj.currentFunnel = currentFunnel;
            
            obj.t_replan = t_replan;
            obj.t_start = t_start;
            obj.x_start = x_start;
            obj.cyclicIdx = cyclicIdx;
            
            obj.currentObstacles = SharedDataHandle(obj.currentObstacles);
            obj.currentFunnel = SharedDataHandle(obj.currentFunnel);
            obj.t_start = SharedDataHandle(obj.t_start);
            obj.x_start = SharedDataHandle(obj.x_start);
            obj.init_replan = SharedDataHandle(false);

	    for k = 1:length(funnelLibrary)
	    	obj.funnelLibraryReduced(k).xyz = funnelLibrary{k}.xyz;
            obj.funnelLibraryReduced(k).cS = funnelLibrary{k}.cS;
            obj.funnelLibraryReduced(k).x0 = funnelLibrary{k}.x0;
            obj.funnelLibraryReduced(k).S0 = funnelLibrary{k}.S0;
	    end

	    disp('Done setting up.');
        end
        
        % control decisions
        function y = output(obj,t,~,x)
            
            % Get index of current funnel
            currentIdx = obj.currentFunnel.getData();
            
            % Get t_current
            t_current = t - obj.t_start.getData();
            
            % Get x_current
            x_current = x;
            x_s = obj.x_start.getData();
            
            % Shift stuff along cyclic dimensions and compute relative state
            x_current(obj.cyclicIdx) = x(obj.cyclicIdx) - x_s(obj.cyclicIdx); % + x0(obj.cyclicIdx);
            
            % Check if it is time to replan (t_current > t_replan)
            
            % If we haven't (re)planned even once yet, do so
            if ~obj.init_replan.getData()
                replan = true;
                obj.init_replan.setData(true);
            elseif t_current > obj.t_replan
                replan = true;
            else % If it is not yet time...
                 
                % Check if current state is inside current funnel
                Snow = obj.funnelLibrary{currentIdx}.V.S.fasteval(t_current);
                s1now = obj.funnelLibrary{currentIdx}.V.s1.fasteval(t_current);
                s2now = obj.funnelLibrary{currentIdx}.V.s2.fasteval(t_current);
                insideCurrent = (x_current'*Snow*x_current + s1now'*x_current + s2now < 1);
                     
                % If so, check if current funnel is still collision free
                if insideCurrent
                    collisionFree = obj.isCollisionFree(currentIdx,x_s);
                    
                    % If it is collision free, no need to replan
                    if collisionFree
                        replan = false;
                    else % If it is not collision free anymore, replan
                        disp('current funnel is not collision free')
                        replan = true;
                    end
                    
                else % If not inside current funnel anymore, replan
                    disp('not inside current funnel')
                    replan = true;
                end
            end
            % If we have to replan, do so
            if replan
                % Get next funnel from replanFunnels
                nextFunnel = obj.replanFunnels(t,x);
                % Get t_current (This should really always be 0!)
                t_current = 0; % t - obj.t_start.getData();
                % Get x_current
                x_current = x;
                x_current(obj.cyclicIdx) = zeros(length(obj.cyclicIdx),1);
                % Compute correct control input
                y = obj.funnelLibrary{nextFunnel}.controller.output(t_current,[],x_current);
            else % If no replanning necessary, follow current plan
                % Compute correct control input
                y = obj.funnelLibrary{currentIdx}.controller.output(t_current,[],x_current);
            end
                        
        end
        
        function nextFunnel = replanFunnels(obj,t,x)
           % Get latest reported obstacle positions
           obstacles = obj.currentObstacles.getData();
           nextFunnel = replanFunnels_mex(x,obstacles,obj.funnelLibraryReduced);
            
           % Set start time for new funnels
           obj.t_start.setData(t);
            
           % Set start state for new funnels
           obj.x_start.setData(x);
            
           % Set current funnel to new planned funnel
           obj.currentFunnel.setData(nextFunnel);
        end

	% Checks if a given funnel is collision free if executed beginning
        % at x (assuming polytopic obstacles) with bullet collision checking
        function [collisionFree,min_dist] = isCollisionFree(obj,funnelIdx,x)
            % Get latest reported obstacle positions
            obstacles = obj.currentObstacles.getData();
            [collisionFree,min_dist] = isCollisionFree_mex(x,obstacles,obj.funnelLibraryReduced, funnelIdx);    
    	end	      
        
    end
    
end
