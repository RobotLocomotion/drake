classdef QuadraticSumConstraint < QuadraticConstraint
  % lb <= sum_i (x(:,i)-v(:,i))'Qa(x(:,i)-v(:,i)) <= ub
  properties(SetAccess = protected)
    Qa % a matrix of size size(x,1) x size(x,1)
    v % a matrix with same dimension as x
  end
  
  properties(Access = protected)
    x_rows % The number of rows in x
    x_cols % The number of columns in x
  end
  
  methods
    function obj = QuadraticSumConstraint(lb,ub,Qa,v)
      % @param Qa  a matrix of size size(x,1) x size(x,1)
      % @param v   a matrix with same dimension as x
      a_cols = size(v,2);
      Qa_dim = size(Qa,1);
      if(size(v,1) ~= Qa_dim)
        error('Drake:QuadraticSumConstraint:Qa and a have inconsistent dimensions');
      end
      Qa = (Qa+Qa')/2;
      iQ = reshape(repmat(reshape(1:Qa_dim*a_cols,Qa_dim,a_cols),Qa_dim,1),[],1);
      jQ = reshape(bsxfun(@times,(1:Qa_dim*a_cols),ones(Qa_dim,1)),[],1);
      Qval = reshape(bsxfun(@times,Qa(:),ones(1,a_cols)),[],1);
      Q = sparse(iQ,jQ,Qval,a_cols*Qa_dim,a_cols*Qa_dim);
      vQav = sum(sum(v.*(Qa*v)));
      obj = obj@QuadraticConstraint(lb-vQav,ub-vQav,Q*2,-2*Q*v(:));
      obj.Qa = Qa;
      obj.x_rows = Qa_dim;
      obj.x_cols = a_cols;
      obj.v = v;
    end
  end
  
  methods (Access = protected)
    function [c,dc,ddc] = constraintEval(obj,x)
      x = reshape(x,obj.x_rows,obj.x_cols);
      xv = x-obj.v;
      Qxv = obj.Qa*xv;
      c = sum(sum(xv.*Qxv));
      if(nargout>1)
        dc = 2*reshape(Qxv,1,[]);
      end
      if(nargout>2)
        ddc = reshape(obj.Q,1,[]);
      end
    end
  end
  
end