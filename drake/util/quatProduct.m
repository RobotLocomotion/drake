function [q3,dq3] = quatProduct(q1,q2)
% q3 = quatMultiply(q1,q2) Quaternion product of q1 and q
%
% @param q1
% @param q2
% @retval q3 - q1 q2
% @retval dq3 - Jacobian of q3 wrt [q1;q2]

% these are slow and will if they are not met then an error will occur
% immediately below
% sizecheck(q1,[4,1]);  
% sizecheck(q2,[4,1]);

w1 = q1(1);
w2 = q2(1);
v1 = q1(2:4);
v2 = q2(2:4);
crossv1v2 = [v1(2)*v2(3)-v1(3)*v2(2);v1(3)*v2(1)-v1(1)*v2(3);v1(1)*v2(2)-v1(2)*v2(1)];
q3 = [w1*w2 - v1(1)*v2(1)-v1(2)*v2(2)-v1(3)*v2(3); crossv1v2 + w1*v2 + w2*v1];

if nargout > 1
  dq1 = [eye(4),zeros(4)];
  dq2 = [zeros(4),eye(4)];
  dw1 = dq1(1,:);
  dw2 = dq2(1,:);
  dv1 = dq1(2:4,:);
  dv2 = dq2(2:4,:);

  dw3 = w2*dw1 + w1*dw2 - v2'*dv1 - v1'*dv2;
  dv3 = dcross(v1,v2,dv1,dv2) + v2*dw1 + w1*dv2 + v1*dw2 + w2*dv1;

  dq3 = [dw3; dv3];
end

end