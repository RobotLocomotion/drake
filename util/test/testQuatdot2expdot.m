function testQuatdot2expdot
q = uniformlyRandomQuat();
qdot = randn(4,1);
qdot = qdot-q'*qdot*q;
wdot = check_derivative(q,qdot);

q = [1;zeros(3,1)];
qdot = [0;randn(3,1)];
wdot = check_derivative(q,qdot);

N = 50;
q = zeros(4,N);
qdot = randn(4,N);
for i = 1:N
  q(:,i) = uniformlyRandomQuat();
  qdot(:,i) = qdot(:,i)-q(:,i)'*qdot(:,i)*q(:,i);
end
wdot = quatdot2expdot(q,qdot);
for i = 1:N
  valuecheck(check_derivative(q(:,i),qdot(:,i)),wdot(:,i));
end
end

function wdot = check_derivative(q,qdot)
wdot = quatdot2expdot(q,qdot);
w = quat2exp(q);
[w_mex,dw_mex] = quat2expmex(q);
wdot_mex = dw_mex*qdot;
valuecheck(w,w_mex);
valuecheck(wdot,wdot_mex);
omega = quatdot2angularvel(q,qdot);
dt = 1e-5;
angle = norm(omega)*dt;
axis = omega/norm(omega);
R_dt = axis2rotmat([axis;angle]);
q2 = rotmat2quat(R_dt*quat2rotmat(q));
q2 = q2'*q*q2;
valuecheck((quat2exp(q2)-w)/dt,wdot,1e-3);
end