function theta = quatArcDistance(quat1,quat2)
	theta = acos(sum(bsxfun(@times,quat1,quat2),1));
  if theta > pi/2
    theta = pi - theta;
  end
end