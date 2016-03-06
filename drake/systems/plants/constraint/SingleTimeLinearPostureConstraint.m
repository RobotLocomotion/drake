classdef SingleTimeLinearPostureConstraint < RigidBodyConstraint
  % A linear constraint on the robot posture for a single time 
  % lb<=sparse(iAfun,jAvar,A,max(jAvar),robot.nq)*q<=ub
  % @param iAfun      -- The row index of the sparse linear matrix
  % @param jAvar      -- The column index of the sparse linear matrix
  % @param A          -- The non-zero value of the sparse linear matrix
  % @param lb         -- The lower bound of the constraint
  % @param ub         -- The upper bound of the constraint
  % @param num_lcon   -- An integer. The number of linear constraints
  properties(SetAccess = protected)
    iAfun
    jAvar
    A
    lb
    ub
    num_constraint
    A_mat
  end
  
  methods
    function obj = SingleTimeLinearPostureConstraint(robot,iAfun,jAvar,A,lb,ub,tspan)
      if(nargin<7)
        tspan = [-inf,inf];
      end
      
      obj = obj@RigidBodyConstraint(RigidBodyConstraint.SingleTimeLinearPostureConstraintCategory,robot,tspan);
      nq = obj.robot.getNumPositions;
      if(~isnumeric(lb))
        error('Drake:SingleTimeLinearPostureConstraint:lb must be numeric');
      end
      if(~isnumeric(ub))
        error('Drake:SingleTimeLinearPostureConstraint:ub must be numeric');
      end
      obj.num_constraint = length(lb);
      sizecheck(lb,[obj.num_constraint,1]);
      sizecheck(ub,[obj.num_constraint,1]);
      if(any(lb>ub))
        error('Drake:SingleTimeLinearPostureConstraint: lb must be no larger than ub');
      end
      obj.lb = lb;
      obj.ub = ub;
      if(~isnumeric(iAfun))
        error('Drake:SingleTimeLinearPostureConstraint:iAfun must be numeric');
      end
      if(~isnumeric(jAvar))
        error('Drake:SingleTimeLinearPostureConstraint:jAvar must be numeric');
      end
      if(~isnumeric(A))
        error('Drake:SingleTimeLinearPostureConstraint: A must be numeric');
      end
      lenA = length(iAfun);
      sizecheck(iAfun,[lenA,1]);
      sizecheck(jAvar,[lenA,1]);
      if(size(unique([iAfun jAvar],'rows'),1) ~= lenA)
        error('Drake:SingleTimeLinearPostureConstraint: the pair (iAfun(i),jAvar(i)) must be unique');
      end
      if(max(iAfun) ~= obj.num_constraint)
        error('Drake:SingleTimeLinearPostureConstraint: max(iAfun) should be equal to length(lb)');
      end
      if(min(iAfun) ~= 1)
        error('Drake:SingleTimeLinearPostureConstraint: min(iAfun) should be 1');
      end
      if(any(iAfun<=0) || any(iAfun~=floor(iAfun)))
        error('Drake:SingleTimeLinearPostureConstraint: iAfun must be positive integers');
      end
      
      if(max(jAvar)>nq)
        error('Drake:SingleTimeLinearPostureConstraint: jAvar must all be no larger than robot.num_dof');
      end
      if(any(jAvar<=0) || any(jAvar ~= floor(jAvar)))
        error('Drake:SingleTimeLinearPostureConstraint: jAvar must be positive integers');
      end
      obj.iAfun = iAfun;
      obj.jAvar = jAvar;
      obj.A = A;
      obj.A_mat = sparse(iAfun,jAvar,A,obj.num_constraint,nq);
      obj.type = RigidBodyConstraint.SingleTimeLinearPostureConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.SingleTimeLinearPostureConstraintType,robot.getMexModelPtr,iAfun,jAvar,A,lb,ub,tspan);
      end
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
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = obj.lb;
        ub = obj.ub;
      else
        lb = [];
        ub = [];
      end
    end
    
    function c = feval(obj,t,q)
      if(obj.isTimeValid(t))
        c = obj.A_mat*q;
      else
        c = [];
      end
    end
    
    function [iAfun,jAvar,A] = geval(obj,t)
      if(obj.isTimeValid(t))
        iAfun = obj.iAfun;
        jAvar = obj.jAvar;
        A = obj.A;
      else
        iAfun = [];
        jAvar = [];
        A = [];
      end
    end
    
    function [c,dc] = eval(obj,t,q)
      if(obj.isTimeValid(t))
        c = obj.A_mat*q;
        dc = obj.A_mat;
      else
        c = [];
        dc = [];
      end
    end
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = cell(obj.num_constraint,1);
        if(isempty(t))
          t_str = [];
        else
          t_str = sprintf('at time %6.3f',t);
        end
        for i = 1:obj.num_constraint
          name_str{i} = sprintf('SingleTimeLinearPostureConstraint row %d %s',i,t_str);
        end
      else
        name_str = {};
      end
    end
    
    function cnstr = generateConstraint(obj,t)
      if nargin < 2, t = obj.tspan(1); end;
      % generate a LinearConstraint
      if(obj.isTimeValid(t))
        cnstr = {LinearConstraint(obj.lb,obj.ub,obj.A_mat)};
        name_str = obj.name(t);
        cnstr{1} = cnstr{1}.setName(name_str);
      else
        cnstr = {};
      end
    end
  end
end
