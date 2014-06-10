function dHtest
p = RigidBodyManipulator('../Acrobot.urdf',struct('floating',true));
q = randn(p.getNumPositions,1);
v = randn(p.getNumPositions,1);

[H_tv,dH_tv] = geval(@getH,q,v,struct('grad_method','taylorvar'));
[H_mat,dH_mat] = getH_nomex(q,v);
[H_mex,dH_mex] = geval(@getH,q,v);
valuecheck(H_mat,H_tv);
valuecheck(dH_mat,dH_tv);
valuecheck(H_mex,H_tv);
valuecheck(dH_mex,dH_tv);


  function [H,dH] = getH(q,v)
    if (nargout>1)
      [H,C,B,dH,dC,dB] = p.manipulatorDynamics(q,v);
    else
      [H,C,B] = p.manipulatorDynamics(q,v);
    end      
    H = H(:);
  end

  function [H,dH] = getH_nomex(q,v)
    if (nargout>1)
      [H,C,B,dH,dC,dB] = p.manipulatorDynamics(q,v,false);
    else
      [H,C,B] = p.manipulatorDynamics(q,v,false);
    end
    H = H(:);
  end
end