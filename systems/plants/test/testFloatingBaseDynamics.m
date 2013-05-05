function testFloatingBaseDynamics


options.floating = 'rpy';
m_rpy = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);
options.floating = 'YPR';
options.namesuffix = 'ypr_rel'; % floating base uses YPR (with relative/intrinsic angles)
m_ypr_rel = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

% the kinematics and dynamics should actually match, when the order of the indices is
% flopped

ind = [1;2;3;6;5;4];

nq=getNumDOF(m_rpy);
for i=1:25
  q = randn(nq,1); qd = randn(nq,1);

  % check kinematics:
  
  kinsol = doKinematics(m_rpy,q,false,false,qd);
  [pt,J,Jdot] = contactPositionsJdot(m_rpy,kinsol);
  kinsol2 = doKinematics(m_ypr_rel,q(ind),false,false,qd(ind));
  [pt2,J2,Jdot2] = contactPositionsJdot(m_ypr_rel,kinsol2);
  
  valuecheck(pt,pt2);
  valuecheck(J,J2(:,ind));
  valuecheck(Jdot,Jdot2(:,ind));
  
  kinsol = doKinematics(m_rpy,q,false,true,qd);
  [pt2,J2,Jdot2] = contactPositionsJdot(m_rpy,kinsol);
  
  valuecheck(pt,pt2);
  valuecheck(J,J2);
  valuecheck(Jdot,Jdot2);
  
  kinsol2 = doKinematics(m_ypr_rel,q(ind),false,true,qd(ind));
  [pt2,J2,Jdot2] = contactPositionsJdot(m_ypr_rel,kinsol2);

  valuecheck(pt,pt2);
  valuecheck(J,J2(:,ind));
  valuecheck(Jdot,Jdot2(:,ind));
  
  % check dynamics:
  [H,C,B] = manipulatorDynamics(m_rpy,q,qd,false);
  [H2,C2,B2] = manipulatorDynamics(m_ypr_rel,q(ind),qd(ind),false);
  
  valuecheck(H,H2(ind,ind));
  valuecheck(C,C2(ind));
  valuecheck(B,B2(ind,:));

  [H2,C2,B2] = manipulatorDynamics(m_rpy,q,qd,true);
  valuecheck(H,H2);
  valuecheck(C,C2);
  valuecheck(B,B2);

  [H2,C2,B2] = manipulatorDynamics(m_ypr_rel,q(ind),qd(ind),true);

  valuecheck(H,H2(ind,ind));
  valuecheck(C,C2(ind));
  valuecheck(B,B2(ind,:));
  
  xdn = update(m_rpy,0,[q;qd],[]);
  xdn2 = update(m_ypr_rel,0,[q(ind);qd(ind)],[]);
  
  valuecheck(xdn,xdn2([ind;nq+ind]));
end



end