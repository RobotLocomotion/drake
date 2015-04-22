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
w_distance = sum(w_diff.^2,1);
w_distance_flip = zeros(1,N-1);
for i = 2:N
  w_flip = flipExpmap(w(:,i));
  w_distance_flip(i-1) = sum((w_flip-w(:,i-1)).^2,1);
end
if(any(w_distance>w_distance_flip+eps))
  error('quat2expmapSquence chose the exponential map in the wrong direction');
end
end