function sumTest()
  frame = CoordinateFrame('x',5);
  fun1 = drakeFunction.Identity(frame);

  % Test with different inputs
  sum_fun = drakeFunction.Sum({fun1,fun1});
  x = rand(5,1);
  y = rand(5,1);
  [f,df] = sum_fun.eval([x;y]);
  valuecheck(f,x+y);
  valuecheck(df,[eye(frame.dim),eye(frame.dim)]);

  [f,df] = geval(@sum_fun.eval,[x;y],struct('grad_method',{{'user','taylorvar','numerical'}}));

  % Test with same inputs
  sum_fun = drakeFunction.Sum(repmat({fun1},1e1,1),true);
  x = rand(5,1);
  [f,df] = sum_fun.eval(x);
  valuecheck(f,sum_fun.n_contained_functions*x);
  valuecheck(df,sum_fun.n_contained_functions*eye(frame.dim));

  [f,df] = geval(@sum_fun.eval,x,struct('grad_method',{{'user','taylorvar','numerical'}},'tol',1e-6));
end
