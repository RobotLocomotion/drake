function testQuatProduct()
  options.grad_method = {'user','taylorvar','numerical'};
  for i=1:100
    q1 = rand(4,1); q1 = q1/norm(q1);
    q2 = rand(4,1); q2 = q2/norm(q2);
    [~,~] = geval(@quatProduct,q1,q2,options);
    assert(max(abs(quatProduct(q1,q2)-quatmultiply(q1',q2')')) <= eps);
  end
end
