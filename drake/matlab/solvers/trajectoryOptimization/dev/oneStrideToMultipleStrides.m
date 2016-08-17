function xtraj = oneStrideToMultipleStrides(robot,xtraj_stride,n_strides)
  nq = robot.getNumPositions();
  t_stride = xtraj_stride.getBreaks();
  x_stride = xtraj_stride.eval(t_stride);
  q_stride = x_stride(1:nq,:);
  v_stride = x_stride(nq+1:end,:);
  q = q_stride;
  t = t_stride;
  for i = 1:n_strides-1
    q_next = q_stride(:,2:end);
    q_next(1,:) = q_next(1,:) + (q(1,end) - q(1,1));
    q = [q, q_next];
    t = [t, t_stride(2:end) + t(end)];
  end
  v = [v_stride, repmat(v_stride(:,2:end),1,n_strides-1)];
  xtraj = PPTrajectory(foh(t,[q;v]));
  xtraj = xtraj.setOutputFrame(xtraj_stride.getOutputFrame());
end
