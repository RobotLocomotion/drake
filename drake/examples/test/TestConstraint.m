classdef TestConstraint < drakeFunction.DrakeFunction
  % a test constraint for use with the ConstrainedPlant example
  properties
  end
  
  methods
    function obj = TestConstraint()
      obj = obj@drakeFunction.DrakeFunction(2,1);
    end
    
    function [f,df,ddf] = eval(obj,q)
      f = q(1)^2 + 2*q(2)^2 - 3*q(1)*q(2) + 2*q(1) - q(2);
      df = [2*q(1)-3*q(2)+2, 4*q(2)-3*q(1)-1];
      ddf = [2,-3,-3,4];
    end
    
    function [f,J,dJ,Jdotv,dJdotv] = evalWithJdot(obj,q,v)
      nc = 1;
      nq = length(q);
      [f,J,ddf] = eval(obj,q);
      dJ = reshape(ddf,nq*nc,nq);
      Jdotv = ddf*reshape(v*v',nq^2,1);
      %assumes ddf = 0, true for this example
      dJdotv = zeros(nc,2*nq);
      for i=1:nq,
        dJdotv(:,nq+i) = ddf*reshape([zeros(i-1,nq);v';zeros(nq-i,nq)]+[zeros(i-1,nq);v';zeros(nq-i,nq)]',[],1);
      end
    end
  end
  
end