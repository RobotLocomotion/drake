function bullet_collision_zero_rad_sphere_test(varargin)

checkDependency('bullet');

if (nargin > 0)
  typecheck(varargin{1},'double');
  draw_pause = varargin{1};
else
  draw_pause = 0.05;
end

if (nargin > 1)
  typecheck(varargin{2},'double');
  n_points = varargin{2};
else
  n_points = 3;
end

w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = RigidBodyManipulator([],struct('terrain',[]));
for i=1:n_points
  fprintf('Adding point no. %d ...\n',i);
  r = addRobotFromURDF(r,'PointMass.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
end
r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',false));

r.collision_filter_groups('points')=CollisionFilterGroup();
r = addLinksToCollisionFilterGroup(r,repmat({'point'},1,n_points),'points',1:n_points);
r = addToIgnoredListOfCollisionFilterGroup(r,'points','points');
r = r.compile();
warning(w);
v = r.constructVisualizer();

dist_min = 0.0;
dist_max = 0.0;

%c = mycon(randn(r.getNumPositions(),1));  % call it once to make sure it doesn't crash

  function stop=drawme(q,optimValues,state)
    stop=false;
    v.draw(0,[q; 0*q]);


    pause(draw_pause);
  end

  function [c,ceq,GC,GCeq] = mycon(q)
    ceq=[]; GCeq=[];
    kinsol = doKinematics(r,q);
    [c,GC] = closestDistance(r,kinsol);
    c = [dist_min-c;c-dist_max];
    GC = [-GC',GC'];
  end

  function [f,Gf] = myfun(q)
    f = 0;
    Gf = 0*q;
  end

problem.objective = @(q) myfun(q); % feasibility problem
problem.nonlcon = @(q) mycon(q);
problem.solver = 'fmincon';
problem.options=optimset('GradObj','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolFun',1e3,'TolX',1e-14,'TolCon',1e-7,'MaxFunEvals',5000);
%problem.options=optimset('GradConstr','on','Algorithm','sqp','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'TolCon',1e-8,'MaxFunEvals',5000);

tic;
for i=1:10
  q0 = 3*(rand(r.getNumPositions(),1)-0.5);
  v.draw(0,[q0;0*q0]);
  problem.x0 = q0;

  [qsol,~,exitflag] = fmincon(problem);
  success=(exitflag==1 || exitflag==2);

  xyz_idx = bsxfun(@plus,0:6:6*(n_points-1),(1:3)');
  err = bsxfun(@minus,abs(qsol(xyz_idx)),[r.body(1).collision_geometry{1}.size]/2);
  if ~all(any(abs(err)<0.01,1))
    error('Points not on brick!');
  end

end
toc;

end

