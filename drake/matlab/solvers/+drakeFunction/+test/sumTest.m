function sumTest()
  import drakeFunction.*
  dim = 5;
  fun1 = drakeFunction.Identity(dim);

  % Test with different inputs
  sum_fun = Composed(Sum(dim,2),Concatenated({fun1,fun1}));
  x = rand(5,1);
  y = rand(5,1);
  [f,df] = sum_fun([x;y]);
  valuecheck(f,x+y);
  valuecheck(df,[eye(sum_fun.dim_output), eye(sum_fun.dim_output)]);

  [f,df] = geval(@(x) eval(sum_fun,x),[x;y],struct('grad_method',{{'user','taylorvar','numerical'}}));

  % Test with same inputs
  N = 10;
  sum_fun = Composed(Sum(dim,N),Concatenated(repmat({fun1},1,10),true));
  x = rand(5,1);
  [f,df] = sum_fun(x);
  valuecheck(f,N*x);
  valuecheck(df,N*eye(dim));

  [f,df] = geval(@(x) eval(sum_fun,x),x,struct('grad_method',{{'user','taylorvar','numerical'}},'tol',1e-6));
end
