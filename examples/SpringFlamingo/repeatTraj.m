function xtraj_N = repeatTraj(r,xtraj,N,flip_lr)


if flip_lr
  xtraj_flipped = flipLeftRight(r,xtraj);
end

ts_N = xtraj.getBreaks();
xpts_N = xtraj.eval(ts_N);

size_xpts = size(xpts_N);
if length(size_xpts)==3
  % due to PPTrajectory bug...
  xpts_N = reshape(xpts_N, size_xpts(1), size_xpts(3));
end

parity = 1;
for i=1:N
  if flip_lr
    parity = 1-parity;
  end
  
  if parity
    % append input traj
    xtraj_ = xtraj;
  else
    % append flipped traj
    xtraj_ = xtraj_flipped;
  end
  % add to xtraj
  ts = xtraj_.getBreaks();
  xtraj_pts = xtraj_.eval(ts);
  
  T = ts_N(end);
  xT = xpts_N(:,end);
  xtraj_pts(1,:) = xtraj_pts(1,:) + xT(1); % shift pelvis x
  ts = ts + T;
  ts_N = [ts_N, ts];
  size_xpts = size(xtraj_pts);
  if length(size_xpts)==3
    % due to PPTrajectory bug...
    xtraj_pts = reshape(xtraj_pts, size_xpts(1), size_xpts(3));
  end
  xpts_N = [xpts_N, xtraj_pts];

  
end

qpts_N = xpts_N(1:r.getNumPositions,:);
qdpts_N = xpts_N(r.getNumPositions+(1:r.getNumVelocities),:);

qtraj_N = PPTrajectory(foh(ts_N,qpts_N));
qdtraj_N = PPTrajectory(zoh(ts_N,qdpts_N));
xtraj_N = [qtraj_N;qdtraj_N];

end

