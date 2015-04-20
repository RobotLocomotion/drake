function testQuat2expmapSequence()
N = 100;
quat = zeros(4,100);
quat_dot = zeros(4,100);
for i = 1:N
  quat(:,i) = uniformlyRandomQuat();
  quat_dot(:,i) = randn(4,1);
  quat_dot(:,i) = quat_dot(:,i)-quat_dot(:,i)'*quat(:,i)*quat(:,i);
end
[w,wdot] = quat2expmapSequence(quat,quat_dot);
w_diff = diff(w,1,2);
if(any(sqrt(sum(w_diff.^2,1))>2*pi))
  error('The distance between adjacent exponential map should be less than 2*pi');
end
end