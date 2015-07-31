function [xtraj_N,utraj_N,Btraj_N,Ktraj_N,Straj_full_N] = repeatTraj(xtraj,utraj,Btraj,Ktraj,Straj_full,N)

if N<1
  error('invalid N')
elseif N==1
  xtraj_N=xtraj;
  utraj_N=utraj;
  Btraj_N=Btraj;
  Ktraj_N=Ktraj;
  Straj_full_N=Straj_full;
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

m = length(Ktraj);
k=m;

Btraj_N = Btraj;
Ktraj_N = Ktraj;
Straj_full_N = Straj_full;

xtraj_N = xtraj;
utraj_N = utraj;
  
for i=1:N-1
  % add to xtraj
  T = xtraj_N.tspan(2);
  xT = xtraj_N.eval(T);

  xT_1 = 0*xT; xT_1(1)=xT(1); % grab pelvis x

  xtraj_ = xtraj + xT_1;
  xtraj_ = shiftTime(xtraj_,T);
  xtraj_N = xtraj_N.append(xtraj_);
  
  utraj_ = shiftTime(utraj,T);
  utraj_N = utraj_N.append(utraj_);
  
  for j=1:m
    Btraj_N{k+j} = shiftTime(Btraj{j},T);
    Ktraj_N{k+j} = shiftTime(Ktraj{j},T);
    Straj_full_N{k+j} = shiftTime(Straj_full{j},T);
  end
  k=k+m;
end

end

