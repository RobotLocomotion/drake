function sumTest()
  import drakeFunction.*
  frame = CoordinateFrame('x',5);
  fun1 = drakeFunction.Identity(frame);

  % Test with different inputs
  sum_fun = Composed(Sum(frame,2),Concatenated({fun1,fun1}));
  x = rand(5,1);
  y = rand(5,1);
  [f,df] = sum_fun.eval([x;y]);
  valuecheck(f,x+y);
  valuecheck(df,[eye(frame.dim),eye(frame.dim)]);

  [f,df] = geval(@sum_fun.eval,[x;y],struct('grad_method',{{'user','taylorvar','numerical'}}));

  % Test with same inputs
  N = 10;
  sum_fun = Composed(Sum(frame,N),Concatenated(repmat({fun1},10,1),true));
  x = rand(5,1);
  [f,df] = sum_fun.eval(x);
  valuecheck(f,N*x);
  valuecheck(df,N*eye(frame.dim));

  [f,df] = geval(@sum_fun.eval,x,struct('grad_method',{{'user','taylorvar','numerical'}},'tol',1e-6));
end
