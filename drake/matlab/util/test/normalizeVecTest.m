function normalizeVecTest
  options.grad_method = {'taylorvar', 'user'};
  x = rand(5,1);
  [~,~,~] = geval(@normalizeVec,x,options);
end