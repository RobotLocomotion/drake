classdef OnlinePlannerMex < DrakeSystem
    
    properties
        % Constant properties
        funnelLibrary;
        funnelLibraryReduced;
        t_replan;
        cyclicIdx;
        
        % Mutable properties
        currentFunnel;
        forestAhead;
        t_start;
        x_start;
        init_replan;
        
    end
    
    methods
        
        % construction
        function obj = OnlinePlannerMex(funnelLibrary,forestAhead,currentFunnel,cyclicIdx,t_replan,t_start,x_start,num_inputs)
            
            disp('Setting up online planner...');
            
            obj = obj@DrakeSystem(0,0,length(x_start),num_inputs,0,0);
            
            obj.funnelLibrary = funnelLibrary;
            
            obj.forestAhead = forestAhead;
            obj.currentFunnel = currentFunnel;
            
            obj.t_replan = t_replan;
            obj.t_start = t_start;
            obj.x_start = x_start;
            obj.cyclicIdx = cyclicIdx;
            
            obj.forestAhead = SharedDataHandleMutable(obj.forestAhead);
            obj.currentFunnel = SharedDataHandleMutable(obj.currentFunnel);
            obj.t_start = SharedDataHandleMutable(obj.t_start);
            obj.x_start = SharedDataHandleMutable(obj.x_start);
            obj.init_replan = SharedDataHandleMutable(false);

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
            % x0 = obj.funnelLibrary(currentIdx).x0(:,1);
            
            % Shift stuff along cyclic dimensions and compute relative
            % state
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
                % insideCurrent = (obj.funnelLibrary(currentIdx).V.eval(t_current,x_current) < 1);
                     
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
                    replan = true;
                end
            end
            % If we have to replan, do so
            if replan
                % keyboard;
                % Get next funnel from replanFunnels
                nextFunnel = obj.replanFunnels(t,x);
                % Get t_current (This should really always be 0!)
                t_current = 0; % t - obj.t_start.getData();
                % Get x_current
                % x_current = x;
                % x_s = obj.x_start.getData();
                % x0 = obj.funnelLibrary(nextFunnel).x0(:,1);
                % Shift stuff along cyclic dimensions and compute relative
                % state
                % x_current(obj.cyclicIdx) = x(obj.cyclicIdx) - x_s(obj.cyclicIdx);
                x_current = x;
                x_current(obj.cyclicIdx) = zeros(length(obj.cyclicIdx),1);
                t
                nextFunnel
                x
                % Compute correct control input
                y = obj.funnelLibrary{nextFunnel}.controller.output(t_current,[],x_current);
            else % If no replanning necessary, follow current plan
                % Compute correct control input
                y = obj.funnelLibrary{currentIdx}.controller.output(t_current,[],x_current);
            end
                        
        end
        
        function nextFunnel = replanFunnels(obj,t,x)
           % Get latest reported obstacle positions
           forest = obj.forestAhead.getData();
           nextFunnel = replanFunnels_mex(x,forest,obj.funnelLibraryReduced);
            
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
            forest = obj.forestAhead.getData();
            
            [collisionFree,min_dist] = isCollisionFree_mex(x,forest,obj.funnelLibraryReduced, funnelIdx);    

	end	
        
        


        %%%%%% Old functions %%%%%

	function nextFunnel = replanFunnelsOld(obj,t,x)
            
            % Go through funnels and find the first collision free one
            
            % If none is found, pick one with smallest objective value
            % (TODO)
            nextFunnel = 1;
            
            N = length(obj.funnelLibrary);

            for k = 1:N
		% Check if we're inside this funnel (TODO). If not, continue.

		% Check if this funnel is collision free
                collFree = obj.isCollisionFreeOld(k,x);
		

                if collFree
                    nextFunnel = k;
                    break;
                end
            end
            
            % Set start time for new funnels
            obj.t_start.setData(t);
            
            % Set start state for new funnels
            obj.x_start.setData(x);
            
            % Set current funnel to new planned funnel
            obj.currentFunnel.setData(nextFunnel);
            
        end
        
        
        % Checks if a given funnel is collision free if executed beginning
        % at x (assuming polytopic obstacles) with bullet collision checking
        function collisionFree = isCollisionFreeOld(obj,funnelIdx,x)
            
            % Get latest reported obstacle positions
            forest = obj.forestAhead.getData();
            
            % Get funnel
            funnel = obj.funnelLibrary(funnelIdx);
            
            % Get time samples
            ts = funnel.ts;
            
            % For each time sample, check if collision free
            collisions = zeros(length(ts),size(forest,2));
            
            for k = 1:length(ts)
                % Check if this particular ellipsoid intersects any
                % polytope in forestAhead
                
                % S = funnel.Sp{k};
                x0k = funnel.x0(:,k);
		cSk = funnel.cS{k};
                
                for j = 1:size(forest,2)

                    % Get vertices of obstacle
		    verts = forest{j};

                    % Shift so that point on trajectory is at origin
                    verts = verts - repmat(x(obj.cyclicIdx) + x0k(obj.cyclicIdx),1,size(verts,2)); % Maybe easier to just do point to distance instead of shifting to origin (be careful of transformation by cS though)
                    
                    % Transform by cholesky factorization of S
                    vertsT = cSk*verts;

                    % Call bullet
		    % dist = ptToPolyDist(vertsT); % Andres's drake bullet code
		    dist = ptToPolyBullet_mex(vertsT);
                    collisions(k,j) = dist < 1;

                end
            end
            
            collisionFree = full(~any(any(collisions)));
                
        end

        % Checks if a given funnel is collision free if executed beginning
        % at x (assuming polytopic obstacles) with cvxgen
        function collisionFree = isCollisionFreeCvx(obj,funnelIdx,x)
            
            % Get latest reported obstacle positions
            forest = obj.forestAhead.getData();
            
            % Get funnel
            funnel = obj.funnelLibrary(funnelIdx);
            
            % Get time samples
            ts = funnel.ts;
            
            % For each time sample, check if collision free
            collisions = zeros(length(ts),size(forest,2));
            
            x0 = funnel.x0(:,1); % Start of traj
            
            for k = 1:length(ts)
                % Check if this particular ellipsoid intersects any
                % polytope in forestAhead
                
                S = funnel.Sp{k};
                s1 = funnel.s1p{k};
                s2 = funnel.s2p{k};
                x0k = funnel.x0(:,k);
                
                for j = 1:size(forest,2)
                    Aineq = forest{j}.Aineq;
                    bineq = forest{j}.bineq + Aineq*(x0(obj.cyclicIdx) - x(obj.cyclicIdx)); % ASSUMES OBSTACLES ARE IN CYCLIC DIMENSIONS!
                    

                    collisions(k,j) = obj.checkIntersection(Aineq,bineq,S,s1,s2);
                end
            end
            
            collisionFree = full(~any(any(collisions)));
                
        end
        
        
        
        % Checks if a given ellipse intersects a polytope with cvxgen
        function [collision, optval] = checkIntersection(obj,Aineq,bineq,S,s1,s2)
            
            % Cvxgen setup
            settings.verbose = 0;
            params.A = Aineq;
            params.b = bineq;
            params.S = S; 
            params.s1 = s1;
            params.s2 = s2;

            % Solve with cvxgen
            [~, status] = csolve(params,settings);
            
            % Get optimal value
            optval = status.optval;  sqrt(optval)
            
            % Check if there is a collision
            collision = (optval < 1);
            
        end
        
    end
    
end
            
            
            
            
            
            
