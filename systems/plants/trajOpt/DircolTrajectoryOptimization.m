classdef DircolTrajectoryOptimization < TrajectoryOptimization
  
  properties
  end
  
  methods
    function obj = DircolTrajectoryOptimization(plant,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,varargin)
      obj = obj@TrajectoryOptimization(plant,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,varargin);
    end
    
    function [constraints,dyn_inds] = createDynamicConstraints(obj)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      N = obj.N;
      
      constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);
      
      
      n_vars = 2*nX + 2*nU + 1;
      cfun = @(z) constraint_fun(z(1),z(2:nX+1),z(nX+2:2*nX+1),z(2*nX+2:2*nX+nU+1),z(2*nX+nU+2:2*nX+2*nU+1));
      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,cfun);
      
      for i=1:obj.N-1,        
        dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1)];
        constraints{i} = cnstr;
      end
      
      function [f,df] = constraint_fun(h,x0,x1,u0,u1)
        % wrong way to do it, but lets try it first...
        [xdot0,dxdot0] = obj.plant.dynamics(0,x0,u0);
        [xdot1,dxdot1] = obj.plant.dynamics(0,x1,u1);
        xcol = .5*(x0+x1) + h/8*(xdot0-xdot1);
        dxcol = [1/8*(xdot0-xdot1) (.5*eye(nX) + h/8*dxdot0(:,2:1+nX)) ...
          (.5*eye(nX) - h/8*dxdot1(:,2:1+nX)) h/8*dxdot0(:,nX+2:1+nX+nU) -h/8*dxdot1(:,nX+2:1+nX+nU)];
        xdotcol = -1.5*(x0-x1)/h - .25*(xdot0+xdot1);
        dxdotcol = [1.5*(x0-x1)/h^2 (-1.5*eye(nX)/h - .25*dxdot0(:,2:1+nX)) ...
          (1.5*eye(nX)/h - .25*dxdot1(:,2:1+nX)) -.25*dxdot0(:,nX+2:1+nX+nU) -.25*dxdot1(:,nX+2:1+nX+nU)];
%         n x (1+2n + 2m)
        [g,dgdxcol] = obj.plant.dynamics(0,xcol,.5*(u0+u1));
        dg = dgdxcol(:,2:1+nX)*dxcol + [zeros(nX,1+2*nX) .5*dgdxcol(:,2+nX:1+nX+nU) .5*dgdxcol(:,2+nX:1+nX+nU)];
        f = xdotcol - g;
        df = dxdotcol - dg;
      end
    end
    
    function obj = setupCostFunction(obj,initial_cost,running_cost,final_cost)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      
      if ~isempty(initial_cost)
        obj = obj.addCost(initial_cost,obj.x_inds(:,1));
      end
      
      running_handle = running_cost.getHandle();
      
      running_handle_end = @(z) running_fun_end(z(1),z(2:1+nX),z(2+nX:1+nX+nU));
      running_cost_end = NonlinearConstraint(running_cost.lb,running_cost.ub,1+nX+nU,running_handle_end);
      
      running_handle_mid = @(z) running_fun_mid(z(1),z(2),z(3:2+nX),z(3+nX:2+nX+nU));
      running_cost_mid = NonlinearConstraint(running_cost.lb,running_cost.ub,2+nX+nU,running_handle_mid);
      
      h_ind = obj.h_inds(1);
      x_ind = obj.x_inds(:,1);
      u_ind = obj.u_inds(:,1);
      obj = obj.addCost(running_cost_end,[h_ind;x_ind;u_ind]);
      
      if ~isempty(running_cost)
        for i=2:obj.N-1,
          h_ind = [obj.h_inds(i-1);obj.h_inds(i)];
          x_ind = obj.x_inds(:,i);
          u_ind = obj.u_inds(:,i);
          
          obj = obj.addCost(running_cost_mid,[h_ind;x_ind;u_ind]);
        end        
      end
      
      h_ind = obj.h_inds(end);
      u_ind = obj.u_inds(:,end);
      x_ind = obj.x_inds(:,end);
      
      obj = obj.addCost(running_cost_end,[h_ind;x_ind;u_ind]);

      
      h_ind = obj.h_inds';
      if ~isempty(final_cost)
        obj = obj.addCost(final_cost,[h_ind;x_ind]);
      end
      
      function [f,df] = running_fun_end(h,x,u)
        [f,dg] = running_handle([.5*h;x;u]);
        
        df = [.5*dg(:,1) dg(:,2:end)];
      end
      
      function [f,df] = running_fun_mid(h0,h1,x,u)
        [f,dg] = running_handle([.5*(h0+h1);x;u]);
        
        df = [.5*dg(:,1) .5*dg(:,1) dg(:,2:end)];
      end
    end
    
  end
end