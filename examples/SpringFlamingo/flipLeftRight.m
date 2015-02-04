function xtraj_ = flipLeftRight(r,xtraj)


% keep the same
base_x = findPositionIndices(r,'base_x');
base_z = findPositionIndices(r,'base_z');
base_relative_pitch = findPositionIndices(r,'base_relative_pitch');
back_bky = findPositionIndices(r,'back_bky');

% flip
l_leg = findPositionIndices(r,'l_leg');
r_leg = findPositionIndices(r,'r_leg');

nq = getNumPositions(r);

% periodic constraint
R = zeros(r.getNumStates);
R(2,2) = 1; %z
R(3,3) = 1; %pitch
R(4:6,7:9) = eye(3); %leg joints w/symmetry
R(7:9,4:6) = eye(3); %leg joints w/symmetry
R(10:12,10:12) = eye(3); %x,z,pitch velocities
R(13:15,16:18) = eye(3); %leg joints w/symmetry
R(16:18,13:15) = eye(3); %leg joints w/symmetry

xtraj_ = R*xtraj;
