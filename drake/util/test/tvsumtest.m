function tvsumtest

Q = randn(4);
Q = Q'+Q;  % make it symmetric

for i=1:10
  x=randn(4,1);
%  gradTest(@(x)geval(@quadratic,x,struct('grad_method','taylorvar')),x);
  options.grad_method = {'user','taylorvar'};
  [f,df] = geval(@quadratic,x,options);
end  

function [g,dg] = quadratic(x);
%  g = x'*Q*x;
  g = sum((Q*x).*x,1);
  if (nargout>1)
    dg = 2*x'*Q;
  end
end


end

