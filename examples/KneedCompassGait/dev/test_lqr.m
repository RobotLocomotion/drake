function [c,Ktraj,Straj,Ptraj,Btraj,Ftraj,Straj_full]= test_lqr(constrained_plant,xtraj,utraj,Q,R,Qf)
% Go from a hybrid dircol traj opt to lqr
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;
plant = PlanarRigidBodyManipulator('../KneedCompassGait.urdf',options);
plant = plant.setJointLimits(-inf(6,1),inf(6,1));
plant = plant.compile();

nq = plant.getNumPositions;

ts = xtraj.tspan;
x0 = xtraj.eval(ts(1));
q0 = x0(1:nq);

constrained_plant = convertPlant(plant,constrained_plant,q0);

[c,Ktraj,Straj,Ptraj,Btraj,Ftraj,Straj_full] = constrainedtvlqr(constrained_plant,xtraj,utraj,Q,R,Qf);


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
end
