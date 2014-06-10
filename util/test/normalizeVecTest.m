function normalizeVecTest
  options.grad_method = {'user', 'taylorvar'};
  x = rand(5,1);
  [x_norm,dx_norm,ddx_norm] = geval(@normalizeVec,x,options);
end