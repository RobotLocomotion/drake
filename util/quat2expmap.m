function [w,dw] = quat2expmap(q)
% Given quaternion q, return the exponential representation of the rotation
% matrix, such that axis_angle_i = quat2axis(q(:,i)) and w(:,i) =
% axis(i)*angle(i)
% @param q   A 4 x 1 vector, the quaternion
% @retval w   A 3 x 1 vector. the exponential map representation
assert(isa(q,'numeric'));
assert(all(size(q) == [4,1]));
q = q/norm(q);
is_degenerate = abs(q(1)^2-1)<eps;
if(is_degenerate)
  s = 2;
else
  s = 2*acos(q(1))/sqrt(1-q(1)^2);
end
w = q(2:4)*s;
if(nargout>1)
  if(~is_degenerate)
    dsdq1 = (-2*sqrt(1-q(1)^2)+2*acos(q(1))*q(1))/((1-q(1)^2)^(1.5));
  else
    dsdq1 = 0;
  end
  dw = [q(2)*dsdq1 s 0 0;...
        q(3)*dsdq1 0 s 0;...
        q(4)*dsdq1 0 0 s];
end
end