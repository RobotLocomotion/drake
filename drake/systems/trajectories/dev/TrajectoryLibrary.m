classdef TrajectoryLibrary
% a collection of trajectory objects (all in the same frame) 
% that can answer basic group queries
  
  properties
    frame;
    traj={};
    
    knots=[];          % a dim(frame) x N list of knot points
    knot_parents=[];   % an integer list indicating that knot(i) is associated with traj{knot_parent(i)}
    knot_ind=[];       % an integer indicating that knot(i) is the knot_ind(i)th knot point in trajectory traj{knot_parent(i)}
  end
  
  
  methods
    function obj = TrajectoryLibrary(frame)
      typecheck(frame,'CoordinateFrame');
      obj.frame = frame;
    end
    
    function obj = addTrajectory(obj,traj)
      typecheck(traj,'Trajectory');
      traj = traj.inFrame(obj.frame);
      
      obj.traj{end+1} = traj;
      
      knots = traj.eval(traj.getBreaks());
      obj.knots = [obj.knots, knots];
      obj.knot_parents = [obj.knot_parents, repmat(length(obj.traj),1,size(knots,2))];
      obj.knot_ind = [obj.knot_ind,1:length(knots)];
    end

    function [xnear,traj_ind,traj_time,dxnear] = closestPoint(obj,x)
      nX = length(x);
      N = size(obj.knots,2);
      xbar=x(:,ones(1,N))-obj.knots;
      d=sum(xbar.^2,1);
      [dmin,imin]=min(d);
      traj_ind = obj.knot_parents(imin);
      
      if (nargout>3)
        [xnear,traj_time,dxnear] = closestPoint(obj.traj{traj_ind},x,obj.knot_ind(imin));
      else
        [xnear,traj_time] = closestPoint(obj.traj{traj_ind},x,obj.knot_ind(imin));
      end
    end
    
    function [d,dd] = distanceSq(obj,x)
      if (nargout>1)
        [xnear,~,~,dxnear] = closestPoint(obj,x);
        d = (xnear-x)'*(xnear-x);
        dd = 2*(xnear-x)'*(dxnear - eye(obj.frame.dim));
      else
        xnear = closestPoint(obj,x);
        d = (xnear-x)'*(xnear-x);
      end
    end
    
    function h=fnplt(obj,plotdims)
      h=[];
      for i=1:length(obj.traj)
        h=[h,fnplt(obj.traj(i),plotdims)];
      end
    end
    
  end
end