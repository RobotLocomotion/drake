function testFloatingBaseDynamics


options.floating = 1; % floating base uses RPY (with absolute/extrinsic angles)
m_rpy = RigidBodyManipulator('FallingBrick.urdf',options);
options.floating = -2;
options.namesuffix = 'ypr_rel'; % floating base uses YPR (with relative/intrinsic angles)
m_ypr_rel = RigidBodyManipulator('FallingBrick.urdf',options);

% the kinematics and dynamics should actually match, when the order of the indices is
% flopped

ind = [1;2;3;6;5;4];

nq=getNumDOF(m_rpy);
for i=1:25
  q = randn(nq,1); qd = randn(nq,1);

  % check kinematics:
  
  kinsol = doKinematics(m_rpy,q,false,false,qd);
  pt = contactPositions(m_rpy,kinsol);
  kinsol2 = doKinematics(m_ypr_rel,q(ind),false,false,qd(ind));
  pt2 = contactPositions(m_ypr_rel,kinsol2);
  
  valuecheck(pt,pt2);

  
  % check dynamics:
  
  [H,C,B] = manipulatorDynamics(m_rpy,q,qd,false);
  [H2,C2,B2] = manipulatorDynamics(m_ypr_rel,q(ind),qd(ind),false);
  
  valuecheck(H,H2);
  valuecheck(C,C2);
  valuecheck(B,B2);
end



end