function testQuatConjugate()
  options.grad_method = {'user','taylorvar','numerical'};
  for i=1:100
    q = rand(4,1);
    [~,~] = geval(@quatConjugate,q,options);
    assert(max(abs(quatConjugate(q)-quatconj(q')')) < eps);
  end
end
