classdef PostureError < Constraint
  % penalize the cost sum_i (q(:,i)-q_nom(:,i))'*Q*(q(:,i)-q_nom(:,i))
  % @param Q     -- A nq x nq PSD matrix
  % @param q_nom -- A nq x nT double matrix. The nominal posture
  properties
    Q
    q_nom
  end

  methods
    function obj = PostureError(Q,q_nom)
      obj = obj@Constraint(-inf,inf,numel(q_nom),1);
      nq = size(q_nom,1);
      Q = (Q+Q')/2;
      sizecheck(Q,[nq,nq]);
      if(any(eig(Q)<0))
        error('Drake:PostureError:Q must be positive semidefinite');
      end
      obj.Q = Q;
      obj.q_nom = q_nom;
    end
  end

  methods (Access = protected)
    function [c,dc] = constraintEval(obj,x)
      q_err = x-obj.q_nom;
      Qqerr = obj.Q*q_err;
      c = sum(sum(q_err.*Qqerr));
      if nargout>1
        dc = reshape(2*Qqerr,1,[]);
      end
    end
  end
end
