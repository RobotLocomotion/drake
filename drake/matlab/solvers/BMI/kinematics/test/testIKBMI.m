function testIKBMI
checkDependency('spotless');
hand = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/robotiq_simple.urdf'],struct('floating',true));
ik = InverseKinematicsBMI(hand);
hand_tip1 = hand.findLinkId('finger_1_link_3');
hand_tip2 = hand.findLinkId('finger_2_link_3');
hand_tip3 = hand.findLinkId('finger_middle_link_3');
hand_tip_pt1 = [0.01;0.01;0];
hand_tip_pt2 = [0.01;0.01;0];
hand_tip_pt3 = [0.01;0.01;0];

Aobj = [1  0  0; ...
        -1  0  0; ...
        0  1  0; ...
        0 -1  0; ...
        0  0  1; ...
        0  0 -1];
    
bobj = 0.1*ones(size(Aobj,1),1);

% Faces 
% Each face is defined as equality + inequality constraints:
% Aface{i}*[xc;yc;zc] <= bface{i},
% Aface_eq{i}*[xc;yc;zc] = bface_eq{i}.

% Number of faces in our case
N = size(Aobj,1);
Aface_eq = cell(N,1);
bface_eq = cell(N,1);
Aface = cell(N,1);
bface = cell(N,1);
for k = 1:N
    % Inequality constraints
    if mod(k,2) == 1 % if odd
        inds = [1:(k-1),(k+2):N];
    else % if even
        inds = [1:(k-2),(k+1):N];
    end
    Aface{k} = Aobj(inds,:);
    bface{k} = bobj(inds);

    % Equality constraints
    Aface_eq{k} = -Aobj(k,:);
    bface_eq{k} = -bobj(k);   
end
        
face = 1;
ik = ik.addPositionConstraint(hand_tip1,hand_tip_pt1, Aface_eq{face}, bface_eq{face}, Aface{face}, bface{face});
face = 1;
ik = ik.addPositionConstraint(hand_tip2,hand_tip_pt2, Aface_eq{face}, bface_eq{face}, Aface{face}, bface{face});
face = 1;
ik = ik.addPositionConstraint(hand_tip3,hand_tip_pt3, Aface_eq{face}, bface_eq{face}, Aface{face}, bface{face});
ik.plot_iteration = true;
solver_sol = optimize(ik);
sol = ik.retrieveSolution(solver_sol);
kinsol = hand.doKinematics(sol.q);
tip1_pos = hand.forwardKin(kinsol,hand_tip1,hand_tip_pt1);
tip2_pos = hand.forwardKin(kinsol,hand_tip2,hand_tip_pt2);
tip3_pos = hand.forwardKin(kinsol,hand_tip3,hand_tip_pt3);
if(norm(Aface_eq{face}*tip1_pos-bface_eq{face})>1e-3 || any(Aface{face}*tip1_pos-bface{face}>1e-3))
  error('tip1 does not satisfy the kinematics constraint');
end
if(norm(Aface_eq{face}*tip2_pos-bface_eq{face})>1e-3 || any(Aface{face}*tip2_pos-bface{face}>1e-3))
  error('tip2 does not satisfy the kinematics constraint');
end
if(norm(Aface_eq{face}*tip3_pos-bface_eq{face})>1e-3 || any(Aface{face}*tip3_pos-bface{face}>1e-3))
  error('tip3 does not satisfy the kinematics constraint');
end
end
