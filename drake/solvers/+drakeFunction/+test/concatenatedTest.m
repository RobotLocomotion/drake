function concatenatedTest()
  import drakeFunction.*;
  dim1 = 5;
  dim2 = 2;
  fun1 = Sum(dim1,2);
  fun2 = Identity(dim2);
  concatenated_fun = Concatenated({fun1,fun2});
  x = rand(dim1,1);
  y = rand(dim2,1);
  [f,df] = concatenated_fun([x;x;y]);
  valuecheck(f,[x+x;y]);
  valuecheck(df,blkdiag([eye(dim1),eye(dim1)],eye(dim2)));

  % Check [ ; ] syntax
  concatenated_fun = [fun1;fun2];
  x = rand(dim1,1);
  y = rand(dim2,1);
  [f,df] = concatenated_fun([x;x;y]);
  valuecheck(f,[x+x;y]);
  valuecheck(df,blkdiag([eye(dim1),eye(dim1)],eye(dim2)));

  % Check concatenating concatenations
  twice_concatenated_fun = [concatenated_fun;concatenated_fun];
  f2 = twice_concatenated_fun([x;x;y;x;x;y]);
  valuecheck(f2,[f;f]);
end
