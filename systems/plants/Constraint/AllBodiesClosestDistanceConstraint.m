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
        [c,dc] = closestDistanceAllBodies(obj.robot,kinsol);
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
      phi = closestDistanceAllBodies(obj.robot,kinsol);
      obj.num_constraint = numel(phi);
    end

    function drawConstraint(obj,q,lcmgl)
      kinsol = doKinematics(obj.robot,q);
      [ptsA,ptsB,~,dist,~,~,~,idxA,idxB] = closestPointsAllBodies(obj.robot,kinsol);
      draw_idx = find(reshape((dist > obj.ub) | (dist < obj.lb),1,[]));
      for i = draw_idx
        lcmgl.glColor3f(0,0,0); % black

        lcmgl.text(ptsA(:,i),obj.robot.getLinkName(idxA(i)),0,0);
        lcmgl.text(ptsB(:,i),obj.robot.getLinkName(idxB(i)),0,0);

        lcmgl.glBegin( lcmgl.LCMGL_LINES);
        lcmgl.glVertex3f(ptsA(1,i), ptsA(2,i), ptsA(3,i));
        lcmgl.glVertex3f(ptsB(1,i), ptsB(2,i), ptsB(3,i));
        lcmgl.glEnd();

        lcmgl.glColor3f(1,0,0); % red

        lcmgl.sphere(ptsA(:,i),.01,20,20);
        lcmgl.sphere(ptsB(:,i),.01,20,20);

        lcmgl.glColor3f(.7,.7,.7); % gray
      end

      lcmgl.switchBuffers();
    end

    function name = name(obj,t)
      if(obj.isTimeValid(t))
        name = repmat({'Closest distance constraint for all non-adjacent bodies'},obj.getNumConstraint(t),1);
      else
        name = [];
      end
    end

    function ptr = constructPtr(obj)
      ptr = [];
    end
  end
end
