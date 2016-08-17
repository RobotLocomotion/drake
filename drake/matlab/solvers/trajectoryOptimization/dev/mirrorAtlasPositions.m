function q_mirror = mirrorAtlasPositions(robot,q)
  q_mirror = q;

  l_arm_usy_idx = robot.getBody(robot.findJointId('l_arm_usy')).position_num;
  r_arm_usy_idx = robot.getBody(robot.findJointId('r_arm_usy')).position_num;
  q_mirror(l_arm_usy_idx,:) = q(r_arm_usy_idx,:);
  q_mirror(r_arm_usy_idx,:) = q(l_arm_usy_idx,:);

  l_arm_shx_idx = robot.getBody(robot.findJointId('l_arm_shx')).position_num;
  r_arm_shx_idx = robot.getBody(robot.findJointId('r_arm_shx')).position_num;
  q_mirror(l_arm_shx_idx,:) = -q(r_arm_shx_idx,:);
  q_mirror(r_arm_shx_idx,:) = -q(l_arm_shx_idx,:);

  l_arm_ely_idx = robot.getBody(robot.findJointId('l_arm_ely')).position_num;
  r_arm_ely_idx = robot.getBody(robot.findJointId('r_arm_ely')).position_num;
  q_mirror(l_arm_ely_idx,:) = q(r_arm_ely_idx,:);
  q_mirror(r_arm_ely_idx,:) = q(l_arm_ely_idx,:);

  l_arm_elx_idx = robot.getBody(robot.findJointId('l_arm_elx')).position_num;
  r_arm_elx_idx = robot.getBody(robot.findJointId('r_arm_elx')).position_num;
  q_mirror(l_arm_elx_idx,:) = -q(r_arm_elx_idx,:);
  q_mirror(r_arm_elx_idx,:) = -q(l_arm_elx_idx,:);

  l_arm_uwy_idx = robot.getBody(robot.findJointId('l_arm_uwy')).position_num;
  r_arm_uwy_idx = robot.getBody(robot.findJointId('r_arm_uwy')).position_num;
  q_mirror(l_arm_uwy_idx,:) = q(r_arm_uwy_idx,:);
  q_mirror(r_arm_uwy_idx,:) = q(l_arm_uwy_idx,:);

  l_arm_mwx_idx = robot.getBody(robot.findJointId('l_arm_mwx')).position_num;
  r_arm_mwx_idx = robot.getBody(robot.findJointId('r_arm_mwx')).position_num;
  q_mirror(l_arm_mwx_idx,:) = -q(r_arm_mwx_idx,:);
  q_mirror(r_arm_mwx_idx,:) = -q(l_arm_mwx_idx,:);

  l_leg_hpz_idx = robot.getBody(robot.findJointId('l_leg_hpz')).position_num;
  r_leg_hpz_idx = robot.getBody(robot.findJointId('r_leg_hpz')).position_num;
  q_mirror(l_leg_hpz_idx,:) = -q(r_leg_hpz_idx,:);
  q_mirror(r_leg_hpz_idx,:) = -q(l_leg_hpz_idx,:);

  l_leg_hpx_idx = robot.getBody(robot.findJointId('l_leg_hpx')).position_num;
  r_leg_hpx_idx = robot.getBody(robot.findJointId('r_leg_hpx')).position_num;
  q_mirror(l_leg_hpx_idx,:) = -q(r_leg_hpx_idx,:);
  q_mirror(r_leg_hpx_idx,:) = -q(l_leg_hpx_idx,:);

  l_leg_hpy_idx = robot.getBody(robot.findJointId('l_leg_hpy')).position_num;
  r_leg_hpy_idx = robot.getBody(robot.findJointId('r_leg_hpy')).position_num;
  q_mirror(l_leg_hpy_idx,:) = q(r_leg_hpy_idx,:);
  q_mirror(r_leg_hpy_idx,:) = q(l_leg_hpy_idx,:);

  l_leg_kny_idx = robot.getBody(robot.findJointId('l_leg_kny')).position_num;
  r_leg_kny_idx = robot.getBody(robot.findJointId('r_leg_kny')).position_num;
  q_mirror(l_leg_kny_idx,:) = q(r_leg_kny_idx,:);
  q_mirror(r_leg_kny_idx,:) = q(l_leg_kny_idx,:);

  l_leg_akx_idx = robot.getBody(robot.findJointId('l_leg_akx')).position_num;
  r_leg_akx_idx = robot.getBody(robot.findJointId('r_leg_akx')).position_num;
  q_mirror(l_leg_akx_idx,:) = -q(r_leg_akx_idx,:);
  q_mirror(r_leg_akx_idx,:) = -q(l_leg_akx_idx,:);

  l_leg_aky_idx = robot.getBody(robot.findJointId('l_leg_aky')).position_num;
  r_leg_aky_idx = robot.getBody(robot.findJointId('r_leg_aky')).position_num;
  q_mirror(l_leg_aky_idx,:) = q(r_leg_aky_idx,:);
  q_mirror(r_leg_aky_idx,:) = q(l_leg_aky_idx,:);

  base_y = findPositionIndices(robot,'base_y'); base_y = base_y(1);
  q_mirror(base_y,:) = -q(base_y,:);

  base_z = findPositionIndices(robot,'base_z');
  q_mirror(base_z,:) = q(base_z,:);

  base_roll = findPositionIndices(robot,'base_roll');
  q_mirror(base_roll,:) = -q(base_roll,:);

  base_pitch = findPositionIndices(robot,'base_pitch');
  q_mirror(base_pitch,:) = q(base_pitch,:);

  base_yaw = findPositionIndices(robot,'base_yaw');
  q_mirror(base_yaw,:) = -q(base_yaw,:);

  back_bkz = findPositionIndices(robot,'back_bkz');
  q_mirror(back_bkz,:) = -q(back_bkz,:);

  back_bky = findPositionIndices(robot,'back_bky');
  q_mirror(back_bky,:) = q(back_bky,:);

  back_bkx = findPositionIndices(robot,'back_bkx');
  q_mirror(back_bkx,:) = -q(back_bkx,:);
end
