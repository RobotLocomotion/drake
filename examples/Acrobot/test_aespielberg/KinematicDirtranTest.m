classdef KinematicDirtranTest < KinematicTrajectoryOptimization ...
        & DirtranTrajectoryOptimization
    % Trajectory planning problem in which the acceleration of the joints is
    % taken to be directly controllable. The decision variables are the joint
    % postitions (q), velocities (v), and accelerations (u) at each knot point.
    % Linear interpolation is used to enforce the relationships between those
    % quantities as described in DirtranTrajectoryOptimization.
    properties
        v_inds % nv x N array of indices into the decision variable vector
        % corresponding to the velocities at each knot point.
        param_inds %the indices of the parameters
        regularizer_weight
        penalty
    end
    methods
        function obj = KinematicDirtranTest(robot,N,duration,options)
            plant = KinematicDummyPlant(robot);
            parent_args = {plant,N,duration};
            if nargin > 3, parent_args{end+1} = options; end
            obj = obj@DirtranTrajectoryOptimization(parent_args{:});
            obj = obj@KinematicTrajectoryOptimization(robot);
            %obj.v_inds = obj.u_inds;
            obj.v_inds = obj.x_inds(robot.getNumPositions()+(1:robot.getNumVelocities()),:);
        end
        
        %ANDY CODE
        
        function obj = setupVariables(obj,N)
            obj = setupVariables@DirectTrajectoryOptimization(obj,N);
            num_vars = obj.num_vars;
            %ANDYCODE
            params = obj.plant.robot.getParams();
            
            %nq = obj.plant.num_positions;
            %nu = obj.plant.num_u;
            nq = obj.plant.getNumStates() / 2;%TODO: I think this is accurate?
            nu = obj.plant.getNumInputs();
            
            
            
            param_names = params.getFrame().getCoordinateNames();
            obj = obj.addDecisionVariable(length(param_names), param_names);
            obj.param_inds =  (num_vars+1):1:(num_vars+length(param_names));%setup indices
            disp('added params')
        end
        
        function z0 = getInitialVars(obj,t_init,traj_init)
            %ANDYCODE
            z0 = getInitialVars@DirectTrajectoryOptimization(obj,t_init,traj_init);
            z0(obj.param_inds) = obj.plant.robot.getParams().double;
            
        end
        
        function xtraj = reconstructStateTrajectory(obj,z)
            % Interpolate between knot points to reconstruct a trajectory using
            % the hermite spline
            t = [0; cumsum(z(obj.h_inds))];
            u = reshape(z(obj.u_inds),[],obj.N);
            
            x = reshape(z(obj.x_inds),[],obj.N);
            xdot = zeros(size(x,1),obj.N);
            for i=1:obj.N,
                xdot(:,i) = obj.plant.dynamics(t(i),x(:,i),u(:,i));
            end
            xtraj = PPTrajectory(pchipDeriv(t,x,xdot));
            xtraj = xtraj.setOutputFrame(obj.plant.robot.getStateFrame);
        end
        
        
        function obj = addConstraint(obj, constraint, varargin)
            if isa(constraint, 'RigidBodyConstraint')
                obj = addRigidBodyConstraint(obj,constraint, varargin{:});
            else
                obj = addConstraint@DirtranTrajectoryOptimization(obj,constraint,varargin{:});
            end
        end
        
        
        
        function [xtraj,z,F,info] = solveTraj(obj,t_init,q_traj_init)
            %traj_init.x = [q_traj_init.x();q_traj_init.x.fnder()];
            
            %ANDYCODE: make sure to add new parameter constraining cost
            
            obj = obj.addFinalCost(@(T, x, params)obj.parameter_regularizer(T, x, params));
            
            traj_init.x = [q_traj_init.x()];
            traj_init.u = [q_traj_init.x.fnder().fnder()];
            size_u = size(traj_init.u);
            traj_init.u = traj_init.u( (size_u(1)/2 + 1):size_u(1) ); %take just the second half
            [xtraj,~,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
        end
        %ANDYCODE
        function obj = setRegularizerWeight(obj, regularizer_weight)
            obj.regularizer_weight = regularizer_weight;
        end
        
        function obj = setOptimizationType(obj, options)
            obj.penalty = options.penalty;
        end
        
        function [f, df] = parameter_regularizer(obj, T, x, params)
            
            switch obj.penalty
                
                case 1
                    param_values = obj.robot.getParams();
                    w = obj.regularizer_weight;
                    e = 0.00000000000000001;
                    param_diff = (params - double(param_values));
                   
                    
                    f = w*sqrt(param_diff'*param_diff + e);
                    df = [0, zeros(length(x), 1)', w*param_diff' / sqrt(param_diff' * param_diff + e)];
                case 2
                    param_values = obj.robot.getParams();
                    w = obj.regularizer_weight;
                    f = w*norm(double(param_values) - params, 2)^2;
                    df = [0, zeros(length(x), 1)', 2*w*(params - double(param_values))']; 
                    
            end
        end
        
        %ANDYCODE
        function obj = addRunningCost(obj,running_cost_function)
            % Adds an integrated cost to all time steps, which is
            % numerical implementation specific (thus abstract)
            % this cost is assumed to be time-invariant
            % @param running_cost_function a function handle
            %  of the form running_cost_function(dt,x,u)
            
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nP = obj.robot.getNumParams();
            
            for i=1:obj.N-1,
                switch obj.options.integration_method
                    case DirtranTrajectoryOptimization.FORWARD_EULER
                        running_cost = FunctionHandleObjective(1+nX+nU+nP, running_cost_function);
                        inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i); obj.param_inds(:)};
                    case DirtranTrajectoryOptimization.BACKWARD_EULER
                        running_cost = FunctionHandleObjective(1+nX+nU+nP, running_cost_function);
                        inds_i = {obj.h_inds(i);obj.x_inds(:,i+1);obj.u_inds(:,i); obj.param_inds(:)};
                    case DirtranTrajectoryOptimization.MIDPOINT
                        running_cost = FunctionHandleObjective(1+2*nX+2*nU+nP,...
                            @(h,x0,x1,u0,u1,params) obj.midpoint_running_fun(running_cost_function,h,x0,x1,u0,u1,params));
                        inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1); obj.param_inds(:)};
                    otherwise
                        error('Drake:DirtranTrajectoryOptimization:InvalidArgument','Unknown integration method');
                end
                
                obj = obj.addCost(running_cost,inds_i);
            end
        end
        
        %ANDYCODE
        function obj = addFinalCost(obj,final_cost_function)
            % adds a cost to the final state and total time
            % @param final_cost_function a function handle f(T,xf)
            nX = obj.plant.getNumStates();
            nH = obj.N-1;
            nP = obj.robot.getNumParams();
            cost = FunctionHandleObjective(nH+nX+nP,@(h,x, params) obj.final_cost(final_cost_function,h,x, params));
            
            %             cost.grad_method = 'numerical';
            %             cost.grad_level = 0;
            
            obj = obj.addCost(cost,{obj.h_inds;obj.x_inds(:,end); obj.param_inds(:)});
        end
        
        
    end
    
    methods (Access=protected)
        
        function [f,df] = midpoint_running_fun(obj,running_handle,h,x0,x1,u0,u1, params)
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nP = obj.robot.getNumParams();
            [f,dg] = running_handle(h,.5*(x0+x1),.5*(u0+u1), params);
            
            df = [dg(:,1) .5*dg(:,2:1+nX) .5*dg(:,2:1+nX) .5*dg(:,2+nX:1+nX+nU) .5*dg(:,2+nX:1+nX+nU) dg(:,2+nX+nU:1+nX+nU+nP)];
        end
        
        function [f,df] = final_cost(obj,final_cost_function,h,x, params)
            T = sum(h);
            
            %Hopefully the function is either defined here or is passed in
            try
                [f,dg] = final_cost_function(T,x, params);
            catch
                [f,dg] = final_cost_function(T,x, params); %TODO: why did I need this?
            end
            df = [repmat(dg(:,1),1,length(h)) dg(:,2:end)];
        end
    end
    
    
end
