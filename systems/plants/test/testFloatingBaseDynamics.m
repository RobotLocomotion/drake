function testFloatingBaseDynamics

for urdf = {'FallingBrick.urdf',...
    '../../../examples/FurutaPendulum/FurutaPendulum.urdf', ...
    '../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'};

urdf=urdf{1};  
options.floating = 'rpy';
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
m_rpy = TimeSteppingRigidBodyManipulator(urdf,.01,options);
options.floating = 'YPR';
options.namesuffix = 'ypr_rel'; % floating base uses YPR (with relative/intrinsic angles)
m_ypr_rel = TimeSteppingRigidBodyManipulator(urdf,.01,options);
warning(w);

% the kinematics and dynamics should actually match, when the order of the indices is
% flopped

nq=getNumDOF(m_rpy);
ind = [1;2;3;6;5;4;(7:nq)'];
xind = [ind;nq+ind];

x0 = resolveConstraints(m_rpy,Point(getStateFrame(m_rpy)));
x02 = resolveConstraints(m_ypr_rel,Point(getStateFrame(m_ypr_rel)));
valuecheck(x0,x02(xind),1e-4);

w = warning('off','Drake:RigidBodyManipulator:collisionDetect:doKinematicsMex');
for i=1:100
  q = .1*randn(nq,1); qd = randn(nq,1); u = randn(getNumInputs(m_rpy),1);

  kinsol = doKinematics(m_rpy,q,true,false,qd);
  [pt,J,Jdot] = contactPositionsJdot(m_rpy,kinsol);
  [~,~,dJ] = contactPositions(m_rpy,kinsol);
  phi = [jointLimitConstraints(m_rpy,q); contactConstraints(m_rpy,kinsol)];
  
  kinsol2 = doKinematics(m_ypr_rel,q(ind),true,false,qd(ind));
  [pt2,J2,Jdot2] = contactPositionsJdot(m_ypr_rel,kinsol2);
  [~,~,dJ2] = contactPositions(m_ypr_rel,kinsol2);
  
  dJind = reshape(1:nq*nq,[nq,nq]);  
  dJind = reshape(dJind(ind,ind),nq*nq,1); 
  
  valuecheck(pt,pt2);
  valuecheck(J,J2(:,ind));
  valuecheck(Jdot,Jdot2(:,ind));
  valuecheck(full(dJ),full(dJ2(:,dJind))); 
  
  kinsol = doKinematics(m_rpy,q,true,true,qd);
  [pt2,J2,Jdot2] = contactPositionsJdot(m_rpy,kinsol);
  phi2 = [jointLimitConstraints(m_rpy,q); contactConstraints(m_rpy,kinsol)];

  valuecheck(pt,pt2);
  valuecheck(J,J2);
  valuecheck(Jdot,Jdot2);
  valuecheck(phi,phi2);
  
  kinsol2 = doKinematics(m_ypr_rel,q(ind),true,true,qd(ind));
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
  
  xdn = update(m_rpy,0,[q;qd],u);
  xdn2 = update(m_ypr_rel,0,[q(ind);qd(ind)],u);
  
  valuecheck(xdn,xdn2([ind;nq+ind]),1e-4);
end
warning(w);

end

end
