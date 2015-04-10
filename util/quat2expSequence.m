function [w,wdot] = quat2expSequence(quat,quat_dot)
N = size(quat,2);
assert(all(size(quat_dot)==[4,N]));
w = zeros(3,N);
wdot = zeros(3,N);
for i = 1:N
  if(i>1)
    flip = sign(quat(:,i)'*quat(:,i-1));
    quat(:,i) = flip*quat(:,i);
    quat_dot(:,i) = flip*quat_dot(:,i);
  end
  [w(:,i),dw] = quat2expmex(quat(:,i));
  wdot(:,i) = dw*quat_dot(:,i);
end
end