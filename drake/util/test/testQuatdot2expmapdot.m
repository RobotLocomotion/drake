function testQuatdot2expmapdot
q = uniformlyRandomQuat();
qdot = randn(4,1);
qdot = qdot-q'*qdot*q;
check_derivative(q,qdot);

q = [1;zeros(3,1)];
qdot = [0;randn(3,1)];
check_derivative(q,qdot);

N = 50;
q = zeros(4,N);
qdot = randn(4,N);
[w,wdot] = quatdot2expmapdot(q,qdot);
for i = 1:N
  [wi,dwi] = quat2expmap(q(:,i));
  valuecheck(w(:,i),wi);
  valuecheck(wdot(:,i),dwi*qdot(:,i));
end
end

function check_derivative(q,qdot)
[wdot,w] = quatdot2expmapdot(q,qdot);
[w_mex,dw_mex] = quat2expmapmex(q);
wdot_mex = dw_mex*qdot;
valuecheck(w,w_mex);
valuecheck(wdot,wdot_mex);
omega = quatdot2angularvel(q,qdot);
dt = 1e-5;
angle = norm(omega)*dt;
axis = omega/norm(omega);
R_dt = axis2rotmat([axis;angle]);
q2 = rotmat2quat(R_dt*quat2rotmat(q));
q2 = sign(q2'*q)*q2;
valuecheck((quat2expmap(q2)-w)/dt,wdot,1e-3);
end