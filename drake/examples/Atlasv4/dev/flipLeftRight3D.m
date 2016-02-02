function [xtraj_,utraj_,Btraj_,Straj_] = flipLeftRight3D(r,xtraj,utraj,Btraj,Straj)
% specific to planar Atlas model

% keep the same
% base_x = findPositionIndices(r,'base_x');
% base_z = findPositionIndices(r,'base_z');
% base_relative_pitch = findPositionIndices(r,'base_relative_pitch');
% back_bky = findPositionIndices(r,'back_bky');

% flip
% l_leg = findPositionIndices(r,'l_leg');
% r_leg = findPositionIndices(r,'r_leg');

nq = getNumPositions(r);

% R = zeros(r.getNumStates);
% R(1:3,1:3) = eye(3); %x,z,pitch
% R(4:6,8:10) = eye(3); %leg joints w/symmetry
% R(8:10,4:6) = eye(3); %leg joints w/symmetry
% R(7,7) = 1; % back joint
% R(11:13,11:13) = eye(3); %x,z,pitch velocities
% R(14:16,18:20) = eye(3); %leg joints w/symmetry
% R(18:20,14:16) = eye(3); %leg joints w/symmetry
% R(17,17) = 1; % back joint


R = zeros(r.getNumStates);
R(1,1) = 1; %x
R(nq+1,nq+1) = 1; %x
R(2,2) = -1; %y
R(nq+2,nq+2) = -1; %y
R(3,3) = 1; %z
R(nq+3,nq+3) = 1; %z
R(4,4) = -1; %roll
R(nq+4,nq+4) = -1; %roll
R(5,5) = 1; %pitch
R(nq+5,nq+5) = 1; %pitch
R(6,6) = -1; %yaw
R(nq+6,nq+6) = -1; %yaw
R(7:12,13:18) = diag([-1;-1;1;1;1;-1]); %leg joints w/symmetry
R(nq+(7:12),nq+(13:18)) = diag([-1;-1;1;1;1;-1]); %leg joints w/symmetry
R(13:18,7:12) = diag([-1;-1;1;1;1;-1]); %leg joints w/symmetry
R(nq+(13:18),nq+(7:12)) = diag([-1;-1;1;1;1;-1]); %leg joints w/symmetry


% R = zeros(r.getNumStates());
% R(base_x,base_x) = 1;
% R(base_x+nq,base_x+nq) = 1;
% R(base_z,base_z) = 1;
% R(base_z+nq,base_z+nq) = 1;
% R(base_relative_pitch,base_relative_pitch) = 1;
% R(base_relative_pitch+nq,base_relative_pitch+nq) = 1;
% R(back_bky,back_bky) = 1;
% R(back_bky+nq,back_bky+nq) = 1;

% R(l_leg,r_leg) = eye(length(r_leg));
% R(l_leg+nq,r_leg+nq) = eye(length(r_leg));
% R(r_leg,l_leg) = eye(length(r_leg));
% R(r_leg+nq,l_leg+nq) = eye(length(r_leg));

if iscell(xtraj)
  for i=1:length(xtraj),
    xtraj_{i} = R*xtraj{i};
  end
else
  xtraj_ = R*xtraj;
end

for i=1:length(Straj)
  Straj_{i} = R*Straj{i}*R;
end

% INPUTS
% keep the same
% back_bky_in = findInputInd(r,'back_bky');
% flip
l_leg_hpz = findInputInd(r,'l_leg_hpz');
l_leg_hpy = findInputInd(r,'l_leg_hpy');
l_leg_hpx = findInputInd(r,'l_leg_hpx');
l_leg_kny = findInputInd(r,'l_leg_kny');
l_leg_aky = findInputInd(r,'l_leg_aky');
l_leg_akx = findInputInd(r,'l_leg_akx');
r_leg_hpz = findInputInd(r,'r_leg_hpz');
r_leg_hpy = findInputInd(r,'r_leg_hpy');
r_leg_hpx = findInputInd(r,'r_leg_hpx');
r_leg_kny = findInputInd(r,'r_leg_kny');
r_leg_aky = findInputInd(r,'r_leg_aky');
r_leg_akx = findInputInd(r,'r_leg_akx');

U = zeros(r.getNumInputs());
U(l_leg_hpz,r_leg_hpz) = -1; U(r_leg_hpz,l_leg_hpz) = -1;
U(l_leg_hpy,r_leg_hpy) = 1; U(r_leg_hpy,l_leg_hpy) = 1;
U(l_leg_hpx,r_leg_hpx) = -1; U(r_leg_hpx,l_leg_hpx) = -1;
U(l_leg_kny,r_leg_kny) = 1; U(r_leg_kny,l_leg_kny) = 1;
U(l_leg_aky,r_leg_aky) = 1; U(r_leg_aky,l_leg_aky) = 1;
U(l_leg_akx,r_leg_akx) = -1; U(r_leg_akx,l_leg_akx) = -1;

if iscell(utraj)
  for i=1:length(utraj),
    utraj_{i} = U*utraj{i};
  end
else
  utraj_ = U*utraj;
end

for i=1:length(Btraj)
  Btraj_{i} = R*Btraj{i}*U;
end

end

function ind = findInputInd(r,str)
ind = r.getInputFrame.findCoordinateIndex(str);
end