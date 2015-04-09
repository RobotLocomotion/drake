function testQuat2exp
q = uniformlyRandomQuat();
w = quat2exp(q);
a = quat2axis(q);
valuecheck(a(1:3)*a(4),w);

q = [1;0;0;0];
w = quat2exp(q);
valuecheck(w,zeros(3,1));

N = 50;
q = zeros(4,N);
for i = 1:N
  q(:,i) = uniformlyRandomQuat();
end
tic
w = quat2exp(q);
toc
a = zeros(4,N);
tic
for i = 1:N
  a(:,i) = quat2axis(q(:,i));
end
toc
valuecheck(w,a(1:3,:).*bsxfun(@times,ones(3,1),a(4,:)));
end