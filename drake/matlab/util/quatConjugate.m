function [q_conj,dq_conj] = quatConjugate(q)
% q_conj = quatConjugate(q) Quaternion conjugate (with gradient)
%
% @retval q_conj = [w; -v]
% @retval dq_conj - Jacobian of q_conj wrt q

%sizecheck(q,[4,1]);  % this is slow and if it is not true an error will
                      % occur immediately below.
q_conj = [1;-1;-1;-1].*q;

if nargout > 1
  dq_conj = diag([1;-1;-1;-1]);
end
end
