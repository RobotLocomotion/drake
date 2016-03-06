function expmap2quatTest()
  N = 1e2;
  for i = 1:N
    v = -pi + 2*pi*rand(3,1);
    geval_options.grad_method = {'user', 'taylorvar'};
    geval_options.tol = 1e-7;
    [q, dq, ddq] = geval(@(v)expmap2quat(v, false), v, geval_options);
    [q_mex,dq_mex,ddq_mex] = expmap2quat(v, true);
    valuecheck(q,q_mex);
    valuecheck(dq,dq_mex);
    valuecheck(ddq,ddq_mex);
    valuecheck(norm(q), 1);
    
    v = zeros(3,1);
    geval_options.grad_method = {'user', 'numerical'};
    geval_options.tol = 1e-7;
    [q, dq] = geval(@expmap2quat, v, geval_options);
    valuecheck(norm(q), 1);
  end
end
