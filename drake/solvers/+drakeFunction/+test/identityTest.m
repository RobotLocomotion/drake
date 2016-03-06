function identityTest()
  import drakeFunction.*
  dim = 5;
  fcn = Identity(5);
  x = rand(dim,1);
  [f,df] = fcn(x);
  valuecheck(f,x);
  valuecheck(df,eye(dim));
end
