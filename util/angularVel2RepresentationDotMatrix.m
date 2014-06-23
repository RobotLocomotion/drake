function [Phi, dPhidqrot, dPhidq, ddPhidqrotdq] = angularVel2RepresentationDotMatrix(rotation_type, qrot, dqrotdq)
compute_qrot_gradient = nargout > 1;
compute_q_gradient = nargout > 3;
if compute_q_gradient
  nq = size(dqrotdq, 2);
end

switch (rotation_type)
  case 0 % no rotation included
    Phi = zeros(0, 3);
    if compute_qrot_gradient
      dPhidqrot = zeros(numel(Phi), 0);
      if compute_q_gradient
        dPhidq = sparse(numel(Phi), nq);
        ddPhidqrotdq = sparse(numel(dPhidq), nq);
      end
    end
  case 1 % output rpy
    if compute_qrot_gradient
      if compute_q_gradient
        [Phi, dPhidqrot, ddPhidqrotdqrot] = angularvel2rpydotMatrix(qrot);
        dPhidq = dPhidqrot * dqrotdq;
        ddPhidqrotdqrot = reshape(ddPhidqrotdqrot, [], numel(qrot));
        ddPhidqrotdq = ddPhidqrotdqrot * dqrotdq;
      else
        [Phi, dPhidqrot] = angularvel2rpydotMatrix(qrot);
      end
    else
      Phi = angularvel2rpydotMatrix(qrot);
    end
  case 2 % output quaternion
    if compute_qrot_gradient
      [Phi, dPhidqrot] = angularvel2quatdotMatrix(qrot);
      if compute_q_gradient
        dPhidq = dPhidqrot * dqrotdq;
        ddPhidqrotdq = sparse(numel(dPhidqrot), nq);
      end
    else
      Phi = angularvel2quatdotMatrix(qrot);
    end
  otherwise
    error('rotationType not recognized')
end
end