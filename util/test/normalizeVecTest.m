function normalizeVecTest
  options.grad_method = {'user','taylorvar'}
  x = rand(3,1);
  [x_norm,dx_norm] = geval(@normalizeVec,x,options);
end
