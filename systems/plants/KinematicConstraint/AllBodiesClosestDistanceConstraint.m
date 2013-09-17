classdef AllBodiesClosestDistanceConstraint < KinematicConstraint
  properties
    ub
    lb
  end
  methods
    function obj = AllBodiesClosestDistanceConstraint(robot,lb,ub,tspan)
      sizecheck(lb,[1,1]);
      sizecheck(ub,[1,1]);

      obj = obj@KinematicConstraint(robot,tspan);
      obj.lb = lb;
      obj.ub = ub;
      obj = setNumConstraint(obj);
    end

    function [c,dc] = eval(obj,t,kinsol)
      if(obj.isTimeValid(t))
        [c,dc] = closestDistanceForEachBody(obj.robot,kinsol);
      else
        c = [];
        dc = [];
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

    function obj = setNumConstraint(obj)
      kinsol = doKinematics(obj.robot,zeros(obj.robot.getNumDOF(),1));
      phi = closestDistanceForEachBody(obj.robot,kinsol);
      obj.num_constraint = numel(phi);
    end

    function drawConstraint(obj,q,lcmgl)
      kinsol = doKinematics(obj.robot,q);
      [ptsA,ptsB,~,dist,~,~,~,idxA,idxB] = closestPointsForEachBody(obj.robot,kinsol);
      draw_idx = find(reshape((dist > obj.ub) | (dist < obj.lb),1,[]));
      for i = draw_idx
        bot_lcmgl_color3f(lcmgl,0,0,0); % black

        bot_lcmgl_text_ex(lcmgl,ptsA(:,i),obj.robot.getLinkName(idxA(i)),0,0);
        bot_lcmgl_text_ex(lcmgl,ptsB(:,i),obj.robot.getLinkName(idxB(i)),0,0);

        bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
        bot_lcmgl_vertex3f(lcmgl, ptsA(1,i), ptsA(2,i), ptsA(3,i));
        bot_lcmgl_vertex3f(lcmgl, ptsB(1,i), ptsB(2,i), ptsB(3,i));
        bot_lcmgl_end(lcmgl);

        bot_lcmgl_color3f(lcmgl,1,0,0); % red

        bot_lcmgl_sphere(lcmgl,ptsA(:,i),.01,20,20);
        bot_lcmgl_sphere(lcmgl,ptsB(:,i),.01,20,20);

        bot_lcmgl_color3f(lcmgl,.7,.7,.7); % gray
      end

      bot_lcmgl_switch_buffer(lcmgl);
    end

    function name = name(obj,t)
      if(obj.isTimeValid(t))
        name = repmat({'Closest distance constraint for all non-adjacent bodies'},obj.getNumConstraint(t),1);
      else
        name = [];
      end
    end
  end
end
