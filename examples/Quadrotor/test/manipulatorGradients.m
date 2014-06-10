function manipulatorGradients


w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
p = RigidBodyManipulator('../Quadrotor.urdf',struct('floating',true));
warning(w);
q = randn(p.getNumPositions,1);
v = randn(p.getNumPositions,1);

% NOTEST: user gradients aren't actually implemented yet for
% RigidBodyThrust

[H_tv,C_tv,B_tv,dH_tv,dC_tv,dB_tv] = geval(3,@p.manipulatorDynamics,q,v,struct('grad_method','taylorvar'));
[H_mat,C_mat,B_mat,dH_mat,dC_mat,dB_mat] = p.manipulatorDynamics(q,v,false);
[H_mex,C_mex,B_mex,dH_mex,dC_mex,dB_mex] = geval(3,@p.manipulatorDynamics,q,v);
valuecheck(H_mat,H_tv);
valuecheck(C_mat,C_tv);
valuecheck(B_mat,B_tv);
valuecheck(dH_mat,dH_tv);
valuecheck(dC_mat,dC_tv);
valuecheck(dB_mat,dB_tv);
valuecheck(H_mex,H_tv);
valuecheck(C_mex,C_tv);
valuecheck(B_mex,B_tv);
valuecheck(dH_mex,dH_tv);
valuecheck(dC_mex,dC_tv);
valuecheck(dB_mex,dB_tv);


end