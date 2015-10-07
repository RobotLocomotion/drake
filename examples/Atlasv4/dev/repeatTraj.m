function [xtraj_N,utraj_N,Btraj_N,Straj_N] = repeatTraj(r,xtraj,utraj,Btraj,Straj,N,flip_lr,is3D)


if N<1
  error('invalid N')
elseif N==1
  xtraj_N=xtraj;
  utraj_N=utraj;
  Btraj_N=Btraj;
  Straj_N=Straj;
  return
end

if iscell(xtraj)
  xtraj_cell = xtraj;
  xtraj = xtraj_cell{1};
  for i=2:length(xtraj_cell);
    xtraj=xtraj.append(xtraj_cell{i});
  end
end

if iscell(utraj)
  utraj_cell = utraj;
  utraj = utraj_cell{1};
  for i=2:length(utraj_cell);
    utraj=utraj.append(utraj_cell{i});
  end
end

if nargin < 7
  flip_lr = false;
end

if nargin < 7
  is3D = false;
end

if flip_lr
  if is3D
    [xtraj_flipped,utraj_flipped,Btraj_flipped,Straj_flipped] = flipLeftRight3D(r,xtraj,utraj,Btraj,Straj);
  else
    [xtraj_flipped,utraj_flipped,Btraj_flipped,Straj_flipped] = flipLeftRight(r,xtraj,utraj,Btraj,Straj);
  end
end

m = length(Straj);
k=m;

Btraj_N = Btraj;
Straj_N = Straj;
xtraj_N = xtraj;
utraj_N = utraj;

parity = 1;
for i=1:N-1
  if flip_lr
    parity = 1-parity;
  end
  
  if parity
    % append input traj
    xtraj_ = xtraj;
    utraj_ = utraj;
    Btraj_ = Btraj;
    Straj_ = Straj;
  else
    % append flipped traj
    xtraj_ = xtraj_flipped;
    utraj_ = utraj_flipped;
    Btraj_ = Btraj_flipped;
    Straj_ = Straj_flipped;
  end
  
  % add to xtraj
  T = xtraj_N.tspan(2);
  xT = xtraj_N.eval(T);
  
  xT_1 = 0*xT; xT_1(1)=xT(1); % grab pelvis x
  xtraj_tmp = xtraj_ + xT_1;
  xtraj_tmp = shiftTime(xtraj_tmp,T);
  xtraj_N = xtraj_N.append(xtraj_tmp);
  
  utraj_tmp = shiftTime(utraj_,T);
  utraj_N = utraj_N.append(utraj_tmp);
  
  for j=1:m
    Btraj_N{k+j} = shiftTime(Btraj_{j},T);
    Straj_N{k+j} = shiftTime(Straj_{j},T);
  end
  k=k+m;
end

end

