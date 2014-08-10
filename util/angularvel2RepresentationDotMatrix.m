function [Phi, dPhidqrot, dPhidq, ddPhidqrotdq] = angularvel2RepresentationDotMatrix(rotation_type, qrot, dqrotdq)
% Computes the matrix \p Phi that maps an angular velocity vector in world
% frame to the time derivative of a given representation of orientation, as
% specified by \p rotation_type. A useful helper function in functions that
% allow you to specify desired orientation output in various
% representations. Front-end for the various angularvel2* functions.
%
% @param rotation_type 0: no rotation, just return zeros \n
%                      1: rpy representation \n
%                      2: quaternion representation
% @param qrot representation of orientation, matches rotation_type
% @param dqrot gradient of qrot with respect to a coordinate vector q
%
% @retval Phi matrix that maps angular velocity in world frame to time
% derivative of qrot
% @retval dPhidqrot gradient of \p Phi with respect to qrot only
% @retval dPhidq gradient of \p Phi with respect to q
% @retval ddPhidqrotdq gradient of dPhidqrot with respect to q

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
    error('rotation_type not recognized')
end
end