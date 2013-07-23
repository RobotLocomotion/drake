function momentumTest

r = TimeSteppingRigidBodyManipulator('ball.urdf',0.005,struct('floating',true));

x0 = zeros(12,1);
x0(3) = 7;
x0(6+1) = randn();
x0(6+2) = randn();
x0(6+3) = 10*rand();
x0(6+4) = 0.5*randn();
x0(6+5) = 0.5*randn();
x0(6+6) = 0.5*randn();

v = r.constructVisualizer();

  function A = myfun(q)
    % for derivative check
    kinsol = doKinematics(r,q,false);
    A = getCMM(r,kinsol);
  end

T = 3.0;
xtraj = r.simulate([0 T],x0);
lcmgl = bot_lcmgl_init('qp-control-block-debug');

body = getBody(r,2); % get ball

nq = getNumDOF(r);
for t=0:0.05:T
  x = xtraj.eval(t);
  draw(v,t,x);
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

  h = A*qd;

  omega = rpydot2angularvel(q(4:6),qd(4:6));
  am = body.inertia * omega;

  valuecheck(h(1:3),am); 
  valuecheck(h(4:6),body.mass*qd(1:3));
  
  xyzrpy = forwardKin(r,kinsol,2,[0;0;0],1);
  xcom = getCOM(r,kinsol);

  % plot centroidal linear momentum
  bot_lcmgl_push_matrix(lcmgl);
  bot_lcmgl_translated(lcmgl,xcom(1),xcom(2),xcom(3));

  bot_lcmgl_push_matrix(lcmgl);
  axis_rot = rpy2axis(xyzrpy(4:6));
  bot_lcmgl_rotated(lcmgl,axis_rot(4)*180/pi,axis_rot(1),axis_rot(2),axis_rot(3));
  bot_lcmgl_line_width(lcmgl, 1);
  bot_lcmgl_draw_axes(lcmgl);
  bot_lcmgl_pop_matrix(lcmgl);

  % draw linear momentum vector
  bot_lcmgl_line_width(lcmgl, 3);
  bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
  bot_lcmgl_color3f(lcmgl, 1, 1, 0);
  bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
  bot_lcmgl_vertex3f(lcmgl, 0.1*h(4), 0.1*h(5), 0.1*h(6)); % scale for drawing
  bot_lcmgl_end(lcmgl);

  % draw angular momentum vector
  aa_h = rpy2axis(h(1:3));
  bot_lcmgl_line_width(lcmgl, 3);
  bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
  bot_lcmgl_color3f(lcmgl, 0, 1, 1);
  bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
  bot_lcmgl_vertex3f(lcmgl, aa_h(1), aa_h(2), aa_h(3));
  bot_lcmgl_end(lcmgl);

%   % draw angular momentum vector
%   aa_m = rpy2axis(am(1:3));
%   bot_lcmgl_line_width(lcmgl, 3);
%   bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
%   bot_lcmgl_color3f(lcmgl, 1, 0, 0);
%   bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
%   bot_lcmgl_vertex3f(lcmgl, aa_m(1), aa_m(2), aa_m(3));
%   bot_lcmgl_end(lcmgl);

  bot_lcmgl_pop_matrix(lcmgl);
  
  bot_lcmgl_line_width(lcmgl, 1);
  bot_lcmgl_color3f(lcmgl, .2, 0, 0);
  bot_lcmgl_switch_buffer(lcmgl);
  pause(0.1);
end

clear r;
r = TimeSteppingRigidBodyManipulator('brick1.urdf',0.005,struct('floating',true));

x0 = zeros(12,1);
x0(3) = 7;
x0(6+1) = randn();
x0(6+2) = randn();
x0(6+3) = 10*rand();
x0(6+4) = 0.5*randn();
x0(6+5) = 0.5*randn();
x0(6+6) = 0.5*randn();

v = r.constructVisualizer();

T = 3.0;
xtraj = r.simulate([0 T],x0);

body = getBody(r,2); % get brick

nq = getNumDOF(r);
for t=0:0.05:T
  x = xtraj.eval(t);
  draw(v,t,x);
  q = x(1:nq);
  qd = x(nq+(1:nq));
  kinsol = doKinematics(r,q,false,true);
  xyzrpy = forwardKin(r,kinsol,2,[0;0;0],1);
  
  A = getCMM(r,q);
  h = A*qd;
  
  omega = rpydot2angularvel(q(4:6),qd(4:6));
  am = body.inertia * omega;

  valuecheck(h(1:3),am); 
  valuecheck(h(4:6),body.mass*qd(1:3));
  
  xcom = getCOM(r,kinsol);

  % plot centroidal linear momentum
  bot_lcmgl_push_matrix(lcmgl);
  bot_lcmgl_translated(lcmgl,xcom(1),xcom(2),xcom(3));

  bot_lcmgl_push_matrix(lcmgl);
  axis_rot = rpy2axis(xyzrpy(4:6));
  bot_lcmgl_rotated(lcmgl,axis_rot(4)*180/pi,axis_rot(1),axis_rot(2),axis_rot(3));
  bot_lcmgl_line_width(lcmgl, 1);
  bot_lcmgl_draw_axes(lcmgl);
  bot_lcmgl_pop_matrix(lcmgl);

  % draw linear momentum vector
  bot_lcmgl_line_width(lcmgl, 3);
  bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
  bot_lcmgl_color3f(lcmgl, 1, 1, 0);
  bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
  bot_lcmgl_vertex3f(lcmgl, 0.1*h(4), 0.1*h(5), 0.1*h(6)); % scale for drawing
  bot_lcmgl_end(lcmgl);

  % draw angular momentum vector
  aa_h = rpy2axis(h(1:3));
  bot_lcmgl_line_width(lcmgl, 3);
  bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
  bot_lcmgl_color3f(lcmgl, 0, 1, 1);
  bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
  bot_lcmgl_vertex3f(lcmgl, aa_h(1), aa_h(2), aa_h(3));
  bot_lcmgl_end(lcmgl);

%   % draw angular momentum vector
%   aa_m = rpy2axis(am(1:3));
%   bot_lcmgl_line_width(lcmgl, 3);
%   bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
%   bot_lcmgl_color3f(lcmgl, 1, 0, 0);
%   bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
%   bot_lcmgl_vertex3f(lcmgl, aa_m(1), aa_m(2), aa_m(3));
%   bot_lcmgl_end(lcmgl);

  bot_lcmgl_pop_matrix(lcmgl);
  
  bot_lcmgl_line_width(lcmgl, 1);
  bot_lcmgl_color3f(lcmgl, .2, 0, 0);
  bot_lcmgl_switch_buffer(lcmgl);
  pause(0.1);
end

bot_lcmgl_destroy(lcmgl);

end
