function [y,ydot,yddot] = evaluateSplineInLinkConstraints(t,link_constraints,index)

  knot_ind = find(link_constraints(index).ts<=t,1,'last');
  tt = t-link_constraints(index).ts(knot_ind);

  knot_ind = min(knot_ind,length(link_constraints(index).ts)-1);
  coefs = link_constraints(index).coefs(:,knot_ind,:);
  if nargout < 2
    y = evalCubicSplineSegment(tt,coefs);
  elseif nargout < 3
    [y,ydot] = evalCubicSplineSegment(tt,coefs);
  else
    [y,ydot,yddot] = evalCubicSplineSegment(tt,coefs);
  end
end

