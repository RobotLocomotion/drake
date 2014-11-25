classdef SingleTimeKinematicConstraint < RigidBodyConstraint
  % An abstract class. Its eval function take a single time as input, the
  % constraint is enforced at that time only
  % @param num_constraint    -- An int scalar. The number of nonlinear constraints
  properties(SetAccess = protected,GetAccess = protected)
    num_constraint
  end
  
  methods
    function obj = SingleTimeKinematicConstraint(robot,tspan)
      obj = obj@RigidBodyConstraint(RigidBodyConstraint.SingleTimeKinematicConstraintCategory,robot,tspan);
    end
    
    function tspan = getTspan(obj)
      tspan = obj.tspan;
    end
    
    function flag = isTimeValid(obj,t)
      if(isempty(t))
        flag = true;
      else
        if(t>=obj.tspan(1)&&t<=obj.tspan(end))
          flag = true;
        else
          flag = false;
        end
      end
    end
    
    function n = getNumConstraint(obj,t)
      if(obj.isTimeValid(t))
        n = obj.num_constraint;
      else
        n = 0;
      end
    end
    
    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      if(robot.getMexModelPtr~=0 && exist('updatePtrRigidBodyConstraintmex','file'))
        obj.mex_ptr = updatePtrRigidBodyConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
      end
    end
    
    function [c,dc] = eval(obj,t,kinsol)
      if(obj.isTimeValid(t))
        [c,dc] = obj.evalValidTime(kinsol);
      else
        c = [];
        dc = [];
      end
    end
    
    function cnstr = generateConstraint(obj,t)
      % generate a FunctionHandleConstraint object if time is valid or if no time is given
      if nargin < 2, t = obj.tspan(1); end;
      if(obj.isTimeValid(t))
        [lb,ub] = obj.bounds(t);
        cnstr = {FunctionHandleConstraint(lb,ub,obj.robot.getNumPositions,@(~,kinsol) obj.eval(t,kinsol))};
        name_str = obj.name(t);
        cnstr{1} = cnstr{1}.setName(name_str);
        joint_idx = obj.kinematicsPathJoints();
        cnstr{1} = cnstr{1}.setSparseStructure(reshape(bsxfun(@times,(1:obj.num_constraint)',ones(1,length(joint_idx))),[],1),...
          reshape(bsxfun(@times,ones(obj.num_constraint,1),joint_idx),[],1));
      else
        cnstr = {};
      end
    end
    
    function joint_idx = kinematicsPathJoints(obj)
      % return the indices of the joints used to evaluate the constraint. The default
      % value is (1:obj.robot.getNumPositions);
      joint_idx = (1:obj.robot.getNumPositions);
    end
  end
  
  methods(Abstract)
    [lb,ub] = bounds(obj,t)
    name_str = name(obj,t)
  end
  
  methods(Abstract,Access = protected)
    [c,dc] = evalValidTime(obj,kinsol);
  end
end
