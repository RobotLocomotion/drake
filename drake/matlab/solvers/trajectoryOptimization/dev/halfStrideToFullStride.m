function xtraj = halfStrideToFullStride(robot,mirror_fun,xtraj_half)
  % @param mirror_fun   -- function handle that takes an nq x N array of joint
  % positions and returns an appropriately mirrored version. See
  % mirrorAtlasPositions for an example.
  nq = robot.getNumPositions();
  t_half = xtraj_half.getBreaks();
  x_half = xtraj_half.eval(t_half);
  q_half = x_half(1:nq,:);
  v_half = x_half(nq+1:end,:);
  q_mirror = mirror_fun(robot,q_half);
  v_mirror = mirror_fun(robot,v_half);
  q_mirror(1,:) = q_mirror(1,:) + (q_half(1,end) - q_half(1,1));
  q = [q_half, q_mirror(:,2:end)];
  v = [v_half, v_mirror(:,2:end)];
  t = [t_half, t_half(2:end) + t_half(end)];
  xtraj = PPTrajectory(foh(t,[q;v]));
  xtraj = xtraj.setOutputFrame(xtraj_half.getOutputFrame());
end
