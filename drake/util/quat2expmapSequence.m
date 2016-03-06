function [w,wdot] = quat2expmapSequence(quat,quat_dot)
% Given a sequence of quaternions, convert them to exponential map, and
% then unwrap the exponential map such that the adjacent exponential maps
% are close to each other
N = size(quat,2);
assert(all(size(quat_dot)==[4,N]));
w = zeros(3,N);
wdot = zeros(3,N);
for i = 1:N
  [w(:,i),dw] = quat2expmapmex(quat(:,i));
  wdot(:,i) = dw*quat_dot(:,i);
  if(i>1)
    [w(:,i),dw2] = closestExpmap(w(:,i-1),w(:,i));
    wdot(:,i) = dw2*wdot(:,i);
  end
end
end