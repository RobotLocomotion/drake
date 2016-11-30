function momentumTest(display)

if nargin < 1
  display=false;
else
  typecheck(display,'logical');
end

options.floating = 'quat';
options.use_bullet = false;
options.terrain = RigidBodyFlatTerrain();
r = TimeSteppingRigidBodyManipulator('ball.urdf',0.005,options);
if display
  v = r.constructVisualizer();
  checkDependency('lcmgl');
  lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'qp-control-block-debug');
end

manipulator = r.getManipulator();
x0 = [getRandomConfiguration(manipulator); randn(manipulator.getNumVelocities(), 1)];
x0(3) = 15;

T = 3.0;
xtraj = r.simulate([0 T],x0);

body = getBody(r,2); % get ball

nq = getNumPositions(r);
nv = getNumVelocities(r);
for t=0:0.05:T
  x = xtraj.eval(t);
  if display
    draw(v,t,x);
  end
  q = x(1:nq);
  v = x(nq+(1:nv));
  kinsol = doKinematics(r, q, v);
  qd = manipulator.vToqdot(kinsol)*v;
  
  % test derivative
  [A,dAdq] = geval(@(q) gevalFun(r, q) ,q);
  Adot_tv = 0*A;
  for jj=1:nq
    Adot_tv = Adot_tv + reshape(dAdq(:,jj),size(A)) * qd(jj);
  end
  
  A = centroidalMomentumMatrix(r, kinsol);
  Adot_times_v = centroidalMomentumMatrixDotTimesV(r, kinsol);
  valuecheck(Adot_times_v,Adot_tv * v);

  % test mex
  kinsol_matlab = doKinematics(r, q, v, struct('use_mex', false));
  A_mat = centroidalMomentumMatrix(r, kinsol_matlab);
  Adot_times_v_mat = centroidalMomentumMatrixDotTimesV(r, kinsol_matlab);
  valuecheck(A, A_mat);
  valuecheck(Adot_times_v, Adot_times_v_mat);
  
  % test physics
  checkSingleQuatBodyMomentum(r, kinsol, A, v);
  
  if display
    xyzrpy = forwardKin(r,kinsol,2,[0;0;0],1);
    xcom = getCOM(r,kinsol);

    % plot centroidal linear momentum
    lcmgl.glPushMatrix();
    lcmgl.glTranslated(xcom(1),xcom(2),xcom(3));

    lcmgl.glPushMatrix();
    axis_rot = rpy2axis(xyzrpy(4:6));
    lcmgl.glRotated(axis_rot(4)*180/pi,axis_rot(1),axis_rot(2),axis_rot(3));
    lcmgl.glLineWidth(1);
    lcmgl.glDrawAxes();
    lcmgl.glPopMatrix();

    % draw linear momentum vector
    lcmgl.glLineWidth(3);
    lcmgl.glBegin(lcmgl.LCMGL_LINES);
    lcmgl.glColor3f(1, 1, 0);
    lcmgl.glVertex3f(0, 0, 0);
    lcmgl.glVertex3f(0.1*h(4), 0.1*h(5), 0.1*h(6)); % scale for drawing
    lcmgl.glEnd();

    % draw angular momentum vector
    aa_h = rpy2axis(h(1:3));
    lcmgl.glLineWidth(3);
    lcmgl.glBegin(lcmgl.LCMGL_LINES);
    lcmgl.glColor3f(0, 1, 1);
    lcmgl.glVertex3f(0, 0, 0);
    lcmgl.glVertex3f(aa_h(1), aa_h(2), aa_h(3));
    lcmgl.glEnd();

    % draw angular momentum vector
    aa_m = rpy2axis(angular_momentum(1:3));
    lcmgl.glLineWidth(3);
    lcmgl.glBegin(lcmgl.LCMGL_LINES);
    lcmgl.glColor3f(1, 0, 0);
    lcmgl.glVertex3f(0, 0, 0);
    lcmgl.glVertex3f(aa_m(1), aa_m(2), aa_m(3));
    lcmgl.glEnd();

    lcmgl.glPopMatrix();

    lcmgl.glLineWidth(1);
    lcmgl.glColor3f(.2, 0, 0);
    lcmgl.switchBuffers();
    pause(0.1);
  end
end

clear r;
r = TimeSteppingRigidBodyManipulator('brick1.urdf',0.005,options);
manipulator = r.getManipulator();
if display
  v = r.constructVisualizer();
end

x0 = [getRandomConfiguration(manipulator); randn(manipulator.getNumVelocities(), 1)];
x0(3) = 15;

T = 3.0;
xtraj = r.simulate([0 T],x0);

body = getBody(r,2); % get brick

nq = getNumPositions(r);
for t=0:0.05:T
  x = xtraj.eval(t);
  if display
    draw(v,t,x);
  end
  q = x(1:nq);
  v = x(nq+(1:nv));
  kinsol = doKinematics(r,q);
  
  A = centroidalMomentumMatrix(r, kinsol);

  checkSingleQuatBodyMomentum(r, kinsol, A, v);
  
  if display
    xyzrpy = forwardKin(r,kinsol,2,[0;0;0],1);
    xcom = getCOM(r,kinsol);

    % plot centroidal linear momentum
    lcmgl.glPushMatrix();
    lcmgl.glTranslated(xcom(1),xcom(2),xcom(3));

    lcmgl.glPushMatrix();
    axis_rot = rpy2axis(xyzrpy(4:6));
    lcmgl.glRotated(axis_rot(4)*180/pi,axis_rot(1),axis_rot(2),axis_rot(3));
    lcmgl.glLineWidth(1);
    lcmgl.glDrawAxes();
    lcmgl.glPopMatrix();

    % draw linear momentum vector
    lcmgl.glLineWidth(3);
    lcmgl.glBegin(lcmgl.LCMGL_LINES);
    lcmgl.glColor3f(1, 1, 0);
    lcmgl.glVertex3f(0, 0, 0);
    lcmgl.glVertex3f(0.1*h(4), 0.1*h(5), 0.1*h(6)); % scale for drawing
    lcmgl.glEnd();

    % draw angular momentum vector
    aa_h = rpy2axis(h(1:3));
    %lcmgl.glLineWidth(3);
    lcmgl.glBegin(lcmgl.LCMGL_LINES);
    lcmgl.glColor3f(0, 1, 1);
    lcmgl.glVertex3f(0, 0, 0);
    lcmgl.glVertex3f(aa_h(1), aa_h(2), aa_h(3));
    lcmgl.glEnd();

    % draw angular momentum vector
    aa_m = rpy2axis(angular_momentum(1:3));
    lcmgl.glLineWidth(3);
    lcmgl.glBegin(lcmgl.LCMGL_LINES);
    lcmgl.glColor3f(1, 0, 0);
    lcmgl.glVertex3f(0, 0, 0);
    lcmgl.glVertex3f(aa_m(1), aa_m(2), aa_m(3));
    lcmgl.glEnd();

    lcmgl.glPopMatrix();

    lcmgl.glLineWidth(1);
    lcmgl.glColor3f(.2, 0, 0);
    lcmgl.switchBuffers();
    pause(0.1);
  end
end

end

function checkSingleQuatBodyMomentum(r, kinsol, A, v)
h = A*v;
body_id = 2;
body = r.getBody(body_id);
x = forwardKin(r, kinsol, body_id, zeros(3, 1), struct('rotation_type', 2));
quat = x(4 : 7);
R_body_to_world = quat2rotmat(quat);
angular_momentum = R_body_to_world * body.inertia * v(1:3); % in world frame
linear_momentum = R_body_to_world * body.mass * v(4:6);

valuecheck(h(1:3),angular_momentum);
valuecheck(h(4:6),linear_momentum);
end

function A = gevalFun(r, q)
% for derivative check
kinsol = doKinematics(r,q,false);
A = centroidalMomentumMatrix(r, kinsol);
end
