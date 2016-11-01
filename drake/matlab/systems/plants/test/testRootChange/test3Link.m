% Create plant from urdf
p = RigidBodyManipulator('3LinkThing.urdf');
N = p.num_positions; 

% Construct visualizer
v = p.constructVisualizer;
x0 = [0;2*pi/3;-2*pi/3;0;0;0];
v.draw(0,x0);

% Now, get new plants with roots changed
pnew1 = changeRootLink(p, 'link2', [0;0;-1], [0;0;0], []);
pnew2 = changeRootLink(p, 'link3', [0;0;-1], [0;0;0], []);

% permutation matrix
P = diag([-1 -1 1]);  xP = diag([diag(P);diag(P)]);

%pause; 
%vnew1 = pnew1.constructVisualizer;
%clf; vnew1.draw(0,xP*x0);
%pause; 
%vnew2 = pnew2.constructVisualizer;
%clf; vnew2.draw(0,xP*x0);


% Now, compare dynamics at a bunch of points
numTest = 100;
for k = 1:numTest
  x0 = randn(6,1);
  u0 = randn(1,1);
  
  kinsol = doKinematics(p,x0(1:3));
  kinsol_new1 = doKinematics(pnew1,P*x0(1:3));
  kinsol_new2 = doKinematics(pnew2,P*x0(1:3));

  for link = 3:4
    pt = forwardKin(p,kinsol,link,[0;0;-1]);
    ptnew1 = forwardKin(pnew1,kinsol_new1,link,[0;0;-1]);
    ptnew2 = forwardKin(pnew2,kinsol_new2,link,[0;0;-1]);
    valuecheck(pt,ptnew1);
    valuecheck(pt,ptnew2);
  end
  
  xdot_orig = p.dynamics(0,x0,u0*ones(3,1));
  xdotnew1 = xP*pnew1.dynamics(0,xP*x0,P*u0*ones(3,1));
  xdotnew2 = xP*pnew2.dynamics(0,xP*x0,diag([-1 1 -1])*u0*ones(3,1));
    
  valuecheck(xdot_orig,xdotnew1);
  valuecheck(xdot_orig,xdotnew2);
end

disp('Tests passed!');




