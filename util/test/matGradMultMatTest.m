function matGradMultMatTest

options.grad_method = {'taylorvar'};

for i=1:100
  [AB,dAB] = geval(@matTest,randn(2,1),options);
end

end

function [AB,dAB] = matTest(q,options)

  A = rotz(q(1));
  B = rotz(q(2));
  
  AB = A*B;

  if (nargout>1)
    [A,dA] = geval(@rotz,q(1),options);
    [B,dB] = geval(@rotz,q(2),options);
    
    dAB = matGradMultMat(A,B,dA,dB);
  end

end
