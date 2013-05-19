function bullet_collision_jac_test

r = RigidBodyManipulator();

for i=1:2
  r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
end
v = r.constructVisualizer();

q0 = [.1*randn(3,1);randn(3,1);.1*randn(3,1);randn(3,1)];  % two random orientations
v.draw(0,[q0;0*q0]);
c = mycon(q0)

  function stop=drawme(q,optimValues,state)
    stop=false;
    v.draw(0,[q; 0*q]);
  end

  function [c,ceq,GC,GCeq] = mycon(q)
    ceq=[]; GCeq=[];
    kinsol = doKinematics(r,q);
    [c,GC] = pairwiseContactConstraint(r,kinsol,2,3);
    GC = GC';
  end

problem.x0 = [q0;u0;z0];
problem.objective = @(q) 0; % feasibility problem
problem.nonlcon = @(q) mycon(q);
problem.solver = 'fmincon';
problem.options=optimset('GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'TolCon',1e-8,'MaxFunEvals',5000);

[qsol,~,exitflag] = fmincon(problem);
success=(exitflag==1);

if (~success)
  error('failed to resolve bullet contact constraint');
end

end