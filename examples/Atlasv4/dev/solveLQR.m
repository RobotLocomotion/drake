function solveLQR(p,xtraj,utraj,ltraj,Q,R,Qf)

if nargin < 5
  Q = diag([100*ones(p.getNumPositions,1);100*ones(p.getNumVelocities,1)]);
  
%   Q(1:3,1:3) = Q(1:3,1:3)*10;

%was 100,10,.01 changed 4/24/15
  
% 6/3 changed R from .01 to 1 to speed up calculations
  R = 1*eye(getNumInputs(p));
  Qf = 1*Q;
end

% commented out for dircol
options.use_zoh_qd = true;
options.use_zoh_u = true;
options.periodic = true;

R_periodic = zeros(p.getNumStates);
R_periodic(1:3,1:3) = eye(3); %x,z,pitch
R_periodic(4:6,8:10) = eye(3); %leg joints w/symmetry
R_periodic(8:10,4:6) = eye(3); %leg joints w/symmetry
R_periodic(7,7) = 1; % back joint
R_periodic(11:13,11:13) = eye(3); %x,z,pitch velocities
R_periodic(14:16,18:20) = eye(3); %leg joints w/symmetry
R_periodic(18:20,14:16) = eye(3); %leg joints w/symmetry
R_periodic(17,17) = 1; % back joint
options.periodic_jump = R_periodic;
[c,Ktraj,Straj,Ptraj,Btraj,tvec,Straj_full,Ftraj] = hybridconstrainedtvlqr(p,xtraj,utraj,ltraj,Q,R,Qf,options);

keyboard;
% save('data/atlas_passive_ankle_lqr.mat','xtraj','utraj','ltraj','c','Ktraj','Straj','Ptraj','Btraj','tvec','Straj_full','Ftraj','Q','R','Qf');

end

