function momentumTest(display)

if nargin < 1
  display=false;
else
  typecheck(display,'logical');
end

options.floating = true;
options.terrain = RigidBodyFlatTerrain();
r = TimeSteppingRigidBodyManipulator('ball.urdf',0.005,options);
if display
  v = r.constructVisualizer();
  checkDependency('lcmgl');
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'qp-control-block-debug');
end

x0 = zeros(12,1);
x0(3) = 15;
x0(6+1) = randn();
x0(6+2) = randn();
x0(6+3) = 10*rand();
x0(6+4) = 0.5*randn();
x0(6+5) = 0.5*randn();
x0(6+6) = 0.5*randn();

  function A = myfun(q)
    % for derivative check
    kinsol = doKinematics(r,q,false);
    A = getCMM(r,kinsol);
  end

T = 3.0;
xtraj = r.simulate([0 T],x0);

body = getBody(r,2); % get ball

nq = getNumDOF(r);
for t=0:0.05:T
  x = xtraj.eval(t);
  if display
    draw(v,t,x);
  end
  q = x(1:nq);
  qd = x(nq+(1:nq));
  
  % test derivative
  [A,dAdq] = geval(@myfun,q);
  Adot_tv = 0*A;
  for jj=1:nq
    Adot_tv = Adot_tv + reshape(dAdq(:,jj),size(A)) * qd(jj);
  end
  
  kinsol = doKinematics(r,q,false,true);
  [A,Adot] = getCMM(r,kinsol,qd);
  valuecheck(Adot,Adot_tv);

  % test mex
  kinsol_matlab = doKinematics(r,q,false,false);
  [A_mat,Adot_mat] = getCMM(r,kinsol_matlab,qd);
  valuecheck(A,A_mat);
  valuecheck(Adot,Adot_mat);
  
  % test physics
  h = A*qd;
  omega = rpydot2angularvel(q(4:6),qd(4:6));
  am = body.inertia * omega;

  valuecheck(h(1:3),am); 
  valuecheck(h(4:6),body.mass*qd(1:3));
  
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
    aa_m = rpy2axis(am(1:3));
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
if display
  v = r.constructVisualizer();
end

x0 = zeros(12,1);
x0(3) = 15;
x0(6+1) = randn();
x0(6+2) = randn();
x0(6+3) = 10*rand();
x0(6+4) = 0.5*randn();
x0(6+5) = 0.5*randn();
x0(6+6) = 0.5*randn();


T = 3.0;
xtraj = r.simulate([0 T],x0);

body = getBody(r,2); % get brick

nq = getNumDOF(r);
for t=0:0.05:T
  x = xtraj.eval(t);
  if display
    draw(v,t,x);
  end
  q = x(1:nq);
  qd = x(nq+(1:nq));
  kinsol = doKinematics(r,q,false,true);
  
  A = getCMM(r,q);
  h = A*qd;
  
  omega = rpydot2angularvel(q(4:6),qd(4:6));
  am = body.inertia * omega;

  valuecheck(h(1:3),am); 
  valuecheck(h(4:6),body.mass*qd(1:3));

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
    aa_m = rpy2axis(am(1:3));
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
