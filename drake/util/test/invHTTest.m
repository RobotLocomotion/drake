function invHTTest
  rpy = 2*pi*rand(3,1)-pi;
  p = rand(3,1);
  makeT = @(euler_angles,pos) [rpy2rotmat(euler_angles),pos;zeros(1,3),1];
  [T,dT] = geval(makeT,rpy,p,struct('grad_method','taylorvar'));
  dT = jacStd2ht(dT);

  makeInvT = @(euler_angles,pos) invHT(makeT(euler_angles,pos));

  [invT_taylorvar,dinvT_taylorvar] = geval(makeInvT,rpy,p,struct('grad_method','taylorvar'));
  dinvT_taylorvar = jacStd2ht(dinvT_taylorvar);

  [invT,dinvT] = invHT(T,dT);

  assert(all(all(dinvT_taylorvar == dinvT)))
end
