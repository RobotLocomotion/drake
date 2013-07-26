function bullet_collision_closest_distance_jac_test(varargin)

checkDependency('bullet');

if (nargin > 0)
  typecheck(varargin{1},'double');
  draw_pause = varargin{1};
else
  draw_pause = 0.05;
end

lcmgl = BotLCMGLClient('bullet_collision_closest_distance_test');
r = RigidBodyManipulator();

for i=1:2
  r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
end
v = r.constructVisualizer();

dist_min = 0.5;
dist_max = 1;

c = mycon(randn(12,1));  % call it once to make sure it doesn't crash

  function stop=drawme(q,optimValues,state)
    stop=false;
    v.draw(0,[q; 0*q]);

    kinsol = doKinematics(r,q);
    [ptA,ptB,~,dist] = pairwiseClosestPoints(r,kinsol,2,3);
    bot_lcmgl_color3f(lcmgl,1,0,0); % red
    bot_lcmgl_sphere(lcmgl,ptA,.05,20,20);

    bot_lcmgl_color3f(lcmgl,0,1,0); % green
    bot_lcmgl_sphere(lcmgl,ptB,.05,20,20);

    %dist = norm(ptB-ptA);

    bot_lcmgl_color3f(lcmgl,0,0,0); % black
    bot_lcmgl_text_ex(lcmgl,(ptB+ptA)/2,num2str(dist),0,0);

    bot_lcmgl_color3f(lcmgl,.7,.7,.7); % gray

    bot_lcmgl_switch_buffer(lcmgl);


    pause(draw_pause);
  end

  function [c,ceq,GC,GCeq] = mycon(q)
    ceq=[]; GCeq=[];
    kinsol = doKinematics(r,q);
    [c,GC] = pairwiseClosestDistance(r,kinsol,2,3);
    c = [dist_min-c;c-dist_max];
    GC = [-GC',GC'];
  end

problem.objective = @(q) 0; % feasibility problem
problem.nonlcon = @(q) mycon(q);
problem.solver = 'fmincon';
problem.options=optimset('GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'TolCon',1e-7,'MaxFunEvals',5000);

for i=1:10
  q0 = [randn(3,1);randn(3,1);randn(3,1);randn(3,1)];  % two random orientations
  v.draw(0,[q0;0*q0]);
  problem.x0 = q0;

  [qsol,~,exitflag] = fmincon(problem);
  success=(exitflag==1);

  if (~success)
    error('failed to resolve bullet contact constraint');
  end
end

end

