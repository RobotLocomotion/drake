classdef PostureError < NonlinearConstraint
  % penalize the cost sum_i (q(:,i)-q_nom(:,i))'*Q*(q(:,i)-q_nom(:,i))
  % @param Q     -- A nq x nq PSD matrix
  % @param q_nom -- A nq x nT double matrix. The nominal posture
  properties
    Q
    q_nom
  end
  
  methods
    function obj = PostureError(Q,q_nom)
      obj = obj@NonlinearConstraint(-inf,inf,numel(q_nom));
      nq = size(q_nom,1);
      Q = (Q+Q')/2;
      sizecheck(Q,[nq,nq]);
      if(any(eig(Q)<0))
        error('Drake:PostureError:Q must be positive semidefinite');
      end
      obj.Q = Q;
      obj.q_nom = q_nom;
    end
    
    function [c,dc] = eval(obj,x)
      q_err = x-obj.q_nom;
      Qqerr = obj.Q*q_err;
      c = sum(sum(q_err.*Qqerr));
      dc = reshape(2*Qqerr,1,[]);
    end
  end
end