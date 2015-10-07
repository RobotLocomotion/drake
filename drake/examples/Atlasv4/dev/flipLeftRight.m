function [xtraj_,utraj_,Btraj_,Straj_] = flipLeftRight(r,xtraj,utraj,Btraj,Straj)
% specific to planar Atlas model 

% keep the same
base_x = findPositionIndices(r,'base_x');
base_z = findPositionIndices(r,'base_z');
base_relative_pitch = findPositionIndices(r,'base_relative_pitch');
back_bky = findPositionIndices(r,'back_bky');

% flip
l_leg = findPositionIndices(r,'l_leg');
r_leg = findPositionIndices(r,'r_leg');

nq = getNumPositions(r);

R = zeros(r.getNumStates);
R(1:3,1:3) = eye(3); %x,z,pitch
R(4:6,8:10) = eye(3); %leg joints w/symmetry
R(8:10,4:6) = eye(3); %leg joints w/symmetry
R(7,7) = 1; % back joint
R(11:13,11:13) = eye(3); %x,z,pitch velocities
R(14:16,18:20) = eye(3); %leg joints w/symmetry
R(18:20,14:16) = eye(3); %leg joints w/symmetry
R(17,17) = 1; % back joint

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

xtraj_ = R*xtraj;

for i=1:length(Straj)
  Straj_{i} = R*Straj{i}*R;
end

% INPUTS
% keep the same
back_bky_in = findInputInd(r,'back_bky');
% flip
l_leg_in = findInputInd(r,'l_leg');
r_leg_in = findInputInd(r,'r_leg');

U = zeros(r.getNumInputs());
U(back_bky_in,back_bky_in) = 1;
U(l_leg_in,r_leg_in) = eye(length(r_leg_in));
U(r_leg_in,l_leg_in) = eye(length(r_leg_in));

utraj_ = U*utraj;

for i=1:length(Btraj)
  Btraj_{i} = R*Btraj{i}*U;
end

end 

function ind = findInputInd(r,str)
 ind = find(~cellfun('isempty',strfind(r.getInputFrame().getCoordinateNames,str)));
end