function [dH,dH_num,dH_nomex] = dHtest
p = RigidBodyManipulator('../Acrobot.urdf',struct('floating',true));
q = randn(p.getNumPositions,1);
v = randn(p.getNumPositions,1);

[H,dH] = getH(q,v);
[H_num,dH_num] = geval(@getH,q,v,struct('grad_method','taylorvar'));
[H_nomex,dH_nomex] = getH_nomex(q,v);

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