% Go from a hybrid dircol traj opt to lqr
function [c,Ktraj,Straj,Ptraj,Btraj,Straj_full,Ftraj,xtraj,utraj] = test_hybridlqr(constrained_plants,xtraj,utraj,Q,R,Qf,options)
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;
plant = PlanarRigidBodyManipulator('../KneedCompassGait.urdf',options);
plant = plant.setJointLimits(-inf(6,1),inf(6,1));
plant = plant.compile();

nq = plant.getNumPositions;
N = length(constrained_plants);
for i=1:N
  ts_i = xtraj{i}.tspan;
  x0_i = xtraj{i}.eval(ts_i(1));
  q0_i = x0_i(1:nq);
  % Construct reduced order constraints
  new_plant = plant;
  
  constrained_plants{i} = convertPlant(plant,constrained_plants{i},q0_i);
end

% need to exctract hybrid jump, then run in a sequence
for i=2:N,
  jumpfn = @(xm) jump(constrained_plants{i},xm);
  xm = xtraj{i-1}.eval(xtraj{i-1}.tspan(2));
  [~,F] = geval(jumpfn,xm,struct('grad_method','numerical'));
  jump_F{i} = F;
end

if options.periodic
  jump_F{1} = options.periodic_jump;
end

if options.periodic
  Qfi = Qf;
  for loop=1:3,      
    for i=N:-1:1,
      ts = xtraj{i}.tspan;
      [c{i},Ktraj{i},Straj{i},Ptraj{i},Btraj{i},Ftraj{i},Straj_full{i}] = constrainedtvlqr(constrained_plants{i},xtraj{i},utraj{i},Q,R,Qfi);
      P0 = Ptraj{i}.eval(ts(1));
      S0 = Straj{i}.eval(ts(1));
      Qfi = pinv(P0*P0')*P0*S0*P0'*pinv(P0*P0')';  %extract least squares full rank solution
      
      Qfi = jump_F{i}'*Qfi*jump_F{i};
      Qfi = (Qfi+Qfi')/2 + 1e-6*eye(size(Qfi,1));
    end
  end
else
  Qfi = Qf;
  for i=N:-1:1,
    ts = xtraj{i}.tspan;
    [c{i},Ktraj{i},Straj{i},Ptraj{i},Btraj{i},Ftraj{i},Straj_full{i}] = constrainedtvlqr(constrained_plants{i},xtraj{i},utraj{i},Q,R,Qfi);
    P0 = Ptraj{i}.eval(ts(1));
    S0 = Straj{i}.eval(ts(1));
    Qfi = pinv(P0*P0')*P0*S0*P0'*pinv(P0*P0')';  %extract least squares full rank solution
    
    if i~=1
      Qfi = jump_F{i}'*Qfi*jump_F{i};
    end
    Qfi = (Qfi+Qfi')/2 + 1e-6*eye(size(Qfi,1));
  end
end

  function new_plant = convertPlant(plant,constrained_plant,q0)
    % Construct reduced order constraints
    new_plant = plant;
    pos_constraints = constrained_plant.position_constraints;
    J_tmp = zeros(0,nq);
    for j=1:length(pos_constraints)
      [phi,J] = pos_constraints{j}.eval(q0);
      inds_j = [];
      for k=1:length(phi),
        if cond([J_tmp;J(k,:)]) < 100
          J_tmp = [J_tmp;J(k,:)];
          inds_j = [inds_j;k];
        end
      end
      
      old_fun = pos_constraints{j}.fcn;
      
      position_fun = drakeFunction.kinematic.RelativePosition(new_plant,old_fun.frameA,old_fun.frameB,old_fun.pts_in_A,inds_j);
      position_constraint = DrakeFunctionConstraint(pos_constraints{j}.lb,pos_constraints{j}.ub,position_fun);
      position_constraint.grad_level = 2;
      new_plant = new_plant.addPositionEqualityConstraint(position_constraint);
    end    
  end

  function xp = jump(plant,xm)
    %Computes xp, the post impact state of jumping to this constrained plant
    q=xm(1:plant.getNumPositions()); qd=xm((plant.getNumPositions()+1):end);
    H = manipulatorDynamics(plant,q,qd);
    Hinv = inv(H);
      
      
    [phi,J] = plant.positionConstraints(q);
    
    if ~isempty(phi)
      qdp = qd-Hinv*J'*inv(J*Hinv*J')*J*qd;
      xp = [q;qdp];
    else
      xp = xm
    end
  end

end
