classdef PosConstraint < IKconstraint
  properties
    robot
    body
    body_name
    pts
    ub
    lb
    null_const_rows
  end
  
  methods
    function obj = PosConstraint(robot,tspan,body,pts,lb,ub)
      obj = obj@IKconstraint(tspan);
      obj.robot = robot;
      if(typecheck(body,'char'))
        obj.body = robot.findLinkInd(body);
      elseif(typecheck(body,'double'))
        obj.body = body;
      elseif(typecheck(body,'RigidBody'))
        obj.body = robot.findLinkInd(body.linkname);
      else
        error('Drake:posConstraint:Body must be either the link name or the link index');
      end
      obj.body_name = obj.robot.getLink(body).linkname;
      sizecheck(pts,[3,nan]);
      cols = size(pts,2);
      rows = size(lb,1);
      sizecheck(lb,[rows,cols]);
      sizecheck(ub,[rows,cols]);
      if(rows ~=3)
        error('Drake:posConstraint:The rows of ub and lb must be 3');
      end
      lb(isnan(lb)) = -inf;
      ub(isnan(ub)) = inf;
      obj.lb = lb(:);
      obj.ub = ub(:);
      obj.null_const_rows = (isinf(obj.lb)&isinf(obj.ub));
      obj.num_const = sum(~obj.null_const_rows);
    end
    
    function [c,dc] = getConstVal(obj,t,kinsol)
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))||isempty(t)
        [pos,J] = forwardKin(obj.robot,kinsol,obj.body,obj.pts,0);
        c = pos(~obj.null_const_rows);
        dc = J(~obj.null_const_rows,:);
      else
        c = [];
        dc = [];
      end
    end
    
    function [lb,ub] = getConstBnds(obj,t)
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))||isempty(t)
        lb = obj.lb(~obj.null_const_rows);
        ub = obj.ub(~obj.null_const_rows);
      else
        lb = [];
        ub = [];
      end
    end
    
    function name = getConstName(obj,t)
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))||isempty(t)
        name = repmat({sprintf('%s  position constraint at time %10.4f',obj.body_name,t)},3*size(obj.pts,2),1);
      else
        name = [];
      end
    end
  end
end