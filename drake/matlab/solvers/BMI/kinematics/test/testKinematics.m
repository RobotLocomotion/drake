function testKinematics
% test if my kinematics computation is the same as the one in doKinematics
p = RigidBodyManipulator([getDrakePath,'/examples/IRB140/urdf/irb_140_robotiq_simple_ati.urdf'],struct('floating',true));
q = getRandomConfiguration(p);
v = randn(p.getNumVelocities(),1);
q(1:6) = 0;
v(1:6) = 0;
kinsol = p.doKinematics(q,v,struct('use_mex',false));
body_Quat = cell(p.getNumBodies(),1);
body_quat = zeros(4,p.getNumBodies());
body_pos = zeros(3,p.getNumBodies());
for i = 1:p.getNumBodies
  body_pos(:,i) = kinsol.T{i}(1:3,4);
  body_quat(:,i) = rotmat2quat(kinsol.T{i}(1:3,1:3));
  body_Quat{i} = body_quat(:,i)*body_quat(:,i)';
end

for i = 1:p.getNumBodies
  bodyi = p.getBody(i);
  parent_idx = bodyi.parent;
  if(parent_idx ~= 0) % Not 'world'
    if(~bodyi.floating) % ~non-floating base
      if(bodyi.pitch == 0)
        r = bodyi.Ttree(1:3,4);
        pos_expr = rotatePtByQuatBilinear(body_quat(:,parent_idx)*body_quat(:,parent_idx)',r);
        tree_quat = rotmat2quat(bodyi.Ttree(1:3,1:3));
        joint_axis = bodyi.joint_axis;
        qi = q(bodyi.position_num);
        quat_expr = singleBodyQuatPropagation(body_quat(:,parent_idx),tree_quat,joint_axis,cos(qi/2),sin(qi/2));
        forwardKin_expr = [pos_expr;quat_expr];
        fk_bodyi = [body_pos(:,parent_idx);zeros(4,1)]+forwardKin_expr;
        T_fk = [quat2rotmat(fk_bodyi(4:7)) fk_bodyi(1:3);0 0 0 1];
        if(norm(T_fk-kinsol.T{i})>1e-3)
          error('body %s has forward kin error',bodyi.linkname);
        end
      else
        error('not implemented yet');
      end
    else
      T_base = [rotmatFromQuatBilinear(body_Quat{i}) body_pos(:,i);0 0 0 1];
      TJ = bodyi.Ttree*T_base;
      if(norm(rotmat2rpy(TJ(1:3,1:3))-q(4:6))>1e-3)
        error('The rpy for the floating base is not correct');
      end
      if(norm(TJ(1:3,4)-q(1:3))>1e-3)
        error('The position for the floating base is not correct');
      end
    end
  else
  end
end
end