classdef TaskSpaceConstraint < SingleTimeKinematicConstraint
  properties(SetAccess = protected)
    bodyA
    bodyB
    TA
    TB
    pos_max
    pos_min
    rotation_type
    null_const_rows % flags of constraint indicating if it has upper bound equal to infinity, and lower bound equal to minus infinity
    num_const
  end
  methods
    function obj = TaskSpaceConstraint(robot,bodyA,bodyB,TA,TB,pos_min,pos_max,tspan)
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      obj.robot = robot;
      if(typecheck(bodyA,'char'))
        obj.bodyA = robot.findLinkInd(bodyA);
      elseif(typecheck(bodyA,'double'))
        obj.bodyA = bodyA;
      else
        error('bodyA can be either the link name or the link index')
      end
      if(typecheck(bodyB,'char'))
        obj.bodyB = robot.findLinkInd(bodyB);
      elseif(typecheck(bodyB,'double'))
        obj.bodyB = bodyB;
      else
        error('bodyB can be either the link name or the link index')
      end
      isHT(TA);
      isHT(TB);
      obj.TA = TA;
      obj.TB = TB;
      if(any(size(pos_min)~=size(pos_max)))
        error('pos_min and pos_max must have the same dimensions')
      end
      pos_max(isnan(pos_max)) = inf;
      pos_min(isnan(pos_min)) = -inf;
      obj.pos_max = pos_max;
      obj.pos_min = pos_min;
      rows = size(pos_min,1);
      obj.rotation_type = 0*(rows==3)+1*(rows==6)+2*(rows==7);
      obj.null_const_rows = (pos_max == inf)&(pos_min == -inf);
      obj.num_const = sum(~obj.null_const_rows);
    end
    
    function [c,dc] = eval(obj,t,kinsol)
      if ~all(abs(kinsol.q-[obj.body.cached_q]')<1e-8)
        error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not longer valid.  Somebody has called doKinematics with a different q since the solution was computed.  If this happens a lot, I could consider returning the full T tree in kinsol, so I don''t have to rely on this caching mechanism');
      end
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))
        bodyA = obj.body(bodyA);
        bodyB = obj.body(bodyB);
        T_O_A = bodyA.T;
        T_O_B = bodyB.T;
        T_A_B = inv_HT(T_O_A*obj.TA)*T_O_B*TB;
        p = T_A_B(1:3,4);
        R = T_A_B(1:3,1:3);
        if obj.rotation_type == 0
          c = p;
        elseif obj.rotation_type == 1
        elseif obj.rotation_type == 2
        end
      end
    end
    
    function [lb,ub] = bounds(obj,t)
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))
        lb = obj.pos_min(~obj.null_const_rows);
        ub = obj.pos_max(~obj.null_const_rows);
      end
    end
  end
end