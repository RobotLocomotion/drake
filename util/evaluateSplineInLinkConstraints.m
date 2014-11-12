function [y,ydot,yddot] = evaluateSplineInLinkConstraints(t,link_constraints,index)

  knot_ind = find(link_constraints(index).ts<=t,1,'last');
  tt = t-link_constraints(index).ts(knot_ind);

  knot_ind = min(knot_ind,length(link_constraints(index).ts)-1);
  a0 = link_constraints(index).a0(:,knot_ind);
  a1 = link_constraints(index).a1(:,knot_ind);
  a2 = link_constraints(index).a2(:,knot_ind);
  a3 = link_constraints(index).a3(:,knot_ind);
  if nargout < 2
    y = evalCubicSplineSegment(tt,a0,a1,a2,a3);
  elseif nargout < 3
    [y,ydot] = evalCubicSplineSegment(tt,a0,a1,a2,a3);
  else
    [y,ydot,yddot] = evalCubicSplineSegment(tt,a0,a1,a2,a3);
  end
end

