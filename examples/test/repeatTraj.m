function [xtraj_N,utraj_N,Btraj_N,Straj_N,Straj_full_N] = repeatTraj(xtraj,utraj,Btraj,Straj,Straj_full,N)


m = length(Straj);
k=m;

ts_N = xtraj.getBreaks();
xpts_N = xtraj.eval(ts_N);
upts_N = utraj.eval(ts_N);
Btraj_N = Btraj;
Straj_N = Straj;
Straj_full_N = Straj_full;

size_xpts = size(xpts_N);
if length(size_xpts)==3
  % due to PPTrajectory bug...
  xpts_N = reshape(xpts_N, size_xpts(1), size_xpts(3));
end


for i=1:N
  % append input traj
  xtraj_ = xtraj;
  utraj_ = utraj;
  Btraj_ = Btraj;
  Straj_ = Straj;

  % add to xtraj
  ts = xtraj_.getBreaks();
  xtraj_pts = xtraj_.eval(ts);
  utraj_pts = utraj_.eval(ts);
  
  T = ts_N(end);
  xT = xpts_N(:,end);
  xtraj_pts(1,:) = xtraj_pts(1,:) + xT(1); % shift pelvis x
  ts = ts + T;
  ts_N = [ts_N, ts];
  size_xpts = size(xtraj_pts);
  size_upts = size(utraj_pts);
  if length(size_xpts)==3
    % due to PPTrajectory bug...
    xtraj_pts = reshape(xtraj_pts, size_xpts(1), size_xpts(3));
  end
  if length(size_upts)==3
    utraj_pts = reshape(utraj_pts, size_upts(1), size_upts(3));
  end
  xpts_N = [xpts_N, xtraj_pts];
  upts_N = [upts_N, utraj_pts];

  
  for i=1:m
    Btraj_N{k+i} = shiftTime(Btraj_{i},T);
    Straj_N{k+i} = shiftTime(Straj_{i},T);
    Straj_full_N{k+i} = shiftTime(Straj_full{i},T);
  end
  k=k+m;
  
end

xtraj_N = PPTrajectory(foh(ts_N,xpts_N));
utraj_N = PPTrajectory(foh(ts_N,upts_N));

end

