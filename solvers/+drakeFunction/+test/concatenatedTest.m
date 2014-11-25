function concatenatedTest()
  import drakeFunction.*;
  frame1 = CoordinateFrame('x',5);
  frame2 = CoordinateFrame('y',2);
  fun1 = Sum(frame1,2);
  fun2 = Identity(frame2);
  concatenated_fun = Concatenated({fun1,fun2});
  x = rand(frame1.dim,1);
  y = rand(frame2.dim,1);
  [f,df] = concatenated_fun([x;x;y]);
  valuecheck(f,[x+x;y]);
  valuecheck(df,blkdiag([eye(frame1.dim),eye(frame1.dim)],eye(frame2.dim)));

  % Check [ ; ] syntax
  concatenated_fun = [fun1;fun2];
  x = rand(frame1.dim,1);
  y = rand(frame2.dim,1);
  [f,df] = concatenated_fun([x;x;y]);
  valuecheck(f,[x+x;y]);
  valuecheck(df,blkdiag([eye(frame1.dim),eye(frame1.dim)],eye(frame2.dim)));

  % Check concatenating concatenations
  twice_concatenated_fun = [concatenated_fun;concatenated_fun];
  f2 = twice_concatenated_fun([x;x;y;x;x;y]);
  valuecheck(f2,[f;f]);
end
