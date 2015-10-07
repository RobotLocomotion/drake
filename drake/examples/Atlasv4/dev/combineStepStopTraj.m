
t1 = load('data/atlas_step_lqr_sk.mat');
t2 = load('data/atlas_step_stop_lqr_sk.mat');

xtraj1_cell = t1.xtraj;
xtraj1 = xtraj1_cell{1};
utraj1_cell = t1.utraj;
utraj1 = utraj1_cell{1};
for i=2:length(xtraj1_cell);
  xtraj1=xtraj1.append(xtraj1_cell{i});
  utraj1=utraj1.append(utraj1_cell{i});
end

xtraj2_cell = t2.xtraj;
xtraj2 = xtraj2_cell{1};
utraj2_cell = t2.utraj;
utraj2 = utraj2_cell{1};
for i=2:length(xtraj2_cell);
  xtraj2=xtraj2.append(xtraj2_cell{i});
  utraj2=utraj2.append(utraj2_cell{i});
end

T1 = xtraj1.tspan(2);
if ~all(xtraj1.eval(T1)-xtraj2.eval(0) < 1e-5)
  error('final state and starting state do not match')
end
xtraj2 = xtraj2.shiftTime(T1);

% if ~all(utraj1.eval(T1)-utraj2.eval(0) < 1e-5)
%   error('final state and starting input do not match')
% end
utraj2 = utraj2.shiftTime(T1);

xtraj = xtraj1.append(xtraj2);
utraj = utraj1.append(utraj2);

k=length(t1.Btraj);
m=length(t2.Btraj);

Btraj = t1.Btraj;
Ktraj = t1.Ktraj;
Straj = t1.Straj;
Straj_full = t1.Straj_full;

for j=1:m
  Btraj{k+j} = shiftTime(t2.Btraj{j},T1);
  Ktraj{k+j} = shiftTime(t2.Ktraj{j},T1);
  Straj{k+j} = shiftTime(t2.Straj{j},T1);
  Straj_full{k+j} = shiftTime(t2.Straj_full{j},T1);
end

Q=t1.Q;
R=t1.R;
Qf=t1.Qf;


save('data/atlas_step_and_stop_lqr.mat','xtraj','utraj','Ktraj','Straj','Btraj','Straj_full','Q','R','Qf');