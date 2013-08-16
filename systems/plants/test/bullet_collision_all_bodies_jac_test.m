function bullet_collision_all_bodies_jac_test(varargin)

checkDependency('bullet');

if (nargin > 0)
  typecheck(varargin{1},'double');
  draw_pause = varargin{1};
else
  draw_pause = 0.05;
end

if (nargin > 1)
  typecheck(varargin{2},'double');
  n_bricks = varargin{2};
else
  n_bricks = 3;
end

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'bullet_collision_all_bodies_jac_test');
r = RigidBodyManipulator();

for i=1:n_bricks
  fprintf('Adding brick no. %d ...\n',i);
  r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
end
v = r.constructVisualizer();

dist_min = 0.0;
dist_max = 1e3;

%c = mycon(randn(r.getNumDOF(),1));  % call it once to make sure it doesn't crash

  function stop=drawme(q,optimValues,state)
    stop=false;
    v.draw(0,[q; 0*q]);


    pause(draw_pause);
  end

  function [c,ceq,GC,GCeq] = mycon(q)
    ceq=[]; GCeq=[];
    kinsol = doKinematics(r,q);
    [c,GC] = closestDistanceAllBodies(r,kinsol);
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
problem.options=optimset('GradObj','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'TolCon',1e-7,'MaxFunEvals',5000);
%problem.options=optimset('GradConstr','on','Algorithm','sqp','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'TolCon',1e-7,'MaxFunEvals',5000);

tic;
for i=1:10
  q0 = rand(r.getNumDOF(),1);
  v.draw(0,[q0;0*q0]);
  problem.x0 = q0;

  [qsol,~,exitflag] = fmincon(problem);
  success=(exitflag==1);

  if (~success)
    error('failed to resolve bullet contact constraint');
  end
end
toc;

end

