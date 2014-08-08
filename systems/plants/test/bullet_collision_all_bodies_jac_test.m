function bullet_collision_all_bodies_jac_test(varargin)
  rng(1);

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

options.floating = true;
r = RigidBodyManipulator([],options);

for i=1:n_bricks
  fprintf('Adding brick no. %d ...\n',i);
  r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),options);
  %r = addRobotFromURDF(r,'ball.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
end
v = r.constructVisualizer();

dist_min = 2;
dist_max = 1e3;

%c = mycon(randn(r.getNumPositions(),1));  % call it once to make sure it doesn't crash

problem.objective = @(q) myfun(q); % feasibility problem
problem.nonlcon = @(q) mycon(q,r,dist_min,dist_max);
problem.solver = 'fmincon';
problem.options=optimset('GradObj',     'on', ...
                         'GradConstr',  'on', ...
                         'Algorithm',   'interior-point', ...
                         'Display',     'iter', ...
                         'OutputFcn',   @(q,~,~) drawme(q,draw_pause,v), ...
                         'TolX',        1e-14, ...
                         'TolCon',      1e-7, ...
                         'MaxFunEvals', 5000);
%problem.options=optimset('GradConstr','on','Algorithm','sqp','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'TolCon',1e-7,'MaxFunEvals',5000);

tic;
for i=1:10
  %theta = linspace(0,2*pi,n_bricks+1); theta(end) = [];
  %q0 = rand(r.getNumPositions(),1);
  %q0(1:6:end) = 5*cos(theta);
  %q0(2:6:end) = 5*sin(theta);
  %q0(3:6:end) = 0;
  q0 = rand(r.getNumPositions(),1);
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

function stop=drawme(q,draw_pause,v)
  stop=false;
  v.draw(0,[q; 0*q]);
  pause(draw_pause);
end

function [c,ceq,GC,GCeq] = mycon(q,r,dist_min,dist_max)
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


