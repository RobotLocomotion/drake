classdef KinematicPlanner < SimpleDynamicsFullKinematicsPlanner
  methods
    function obj = KinematicPlanner(robot,varargin)
      plant = robot.setNumInputs(robot.getNumVelocities());
      obj = obj@SimpleDynamicsFullKinematicsPlanner(plant,robot,varargin{:});

      % Make timesteps equal
      A_time = [eye(obj.N-2) zeros(obj.N-2,1)] - [zeros(obj.N-2,1) eye(obj.N-2)];
      time_constraint = LinearConstraint(zeros(obj.N-2,1),zeros(obj.N-2,1),A_time);
      obj = obj.addLinearConstraint(time_constraint,obj.h_inds);
    end

    function obj = addDynamicConstraints(obj)
      % obj = addDynamicConstraints(obj,cnstr) adds a dynamic constraint
      % to the planner.
      % @param cnstr  -- Dynamics constraint
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nQ = obj.plant.getNumPositions();
      N = obj.N;
      
      dyn_inds = cell(N-1,1);      
      n_vars = 2*nX + nU + 1;

      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,@constraint_fun);
      
      for i=1:obj.N-1,        
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i)};
        obj = obj.addNonlinearConstraint(cnstr, dyn_inds{i});
      end

      function [f,df] = constraint_fun(h,x0,x1,u)
        xdot = [x1(nQ+(1:nQ));u];
        dxdot = [zeros(nX,nQ+1),eye(nX,nQ),[zeros(nQ);eye(nU)]];
        f = x1 - x0 - h*xdot;
        df = [-xdot -eye(nX) (eye(nX) - h*dxdot(:,2:1+nX)) -h*dxdot(:,nX+2:end)];
      end
    end
    function obj = addRunningCost(obj,running_cost)
      for i=1:obj.N-1,
        h_ind = obj.h_inds(i);
        x_ind = obj.x_inds(:,i);
        u_ind = obj.u_inds(:,i);
        
        obj = obj.addCost(running_cost,{h_ind;x_ind;u_ind});
      end
    end

    function obj = addContactDynamicConstraints(obj)
    end
  end
end
