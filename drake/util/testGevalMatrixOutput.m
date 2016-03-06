function testGevalMatrixOutput()
  % testGevalMatrixOutput checks whether geval runs for numerical
  % differentiation of matrix valued functions.
  fun = @(x) [x(1),0;0,2*x(2)];
  df = reshape([[1,0;0,0],[0,0;0,2]],4,2);
  x0 = zeros(2,1);

  options.grad_method = 'numerical';
  [f,df_num] = geval(fun,x0,options);
  valuecheck(df_num,df);

  options.diff_type = 'forward';
  [f,df_num] = geval(fun,x0,options);
  valuecheck(df_num,df);

  options.diff_type = 'central';
  [f,df_num] = geval(fun,x0,options);
  valuecheck(df_num,df);
end
