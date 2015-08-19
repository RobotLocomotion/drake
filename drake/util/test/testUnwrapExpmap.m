function testUnwrapExpmap()
for i = 1:100
  quat1 = uniformlyRandomQuat();
  quat2 = uniformlyRandomQuat();
  expmap1 = quat2expmap(quat1);
  expmap2 = quat2expmap(quat2);
  options.grad_method = {'taylorvar','user'};
  [expmap2_unwrap,dexpmap2_unwrap] = geval(@(expmap2) unwrapExpmap(expmap1,expmap2),expmap2,options);
  [expmap2_unwrap_mex,dexpmap2_unwrap_mex] = unwrapExpmapmex(expmap1,expmap2);
  valuecheck(expmap2_unwrap,expmap2_unwrap_mex);
  valuecheck(dexpmap2_unwrap,dexpmap2_unwrap_mex);
  expmap2_flip = flipExpmap(expmap2);
  unwrap_distance = norm(expmap2_unwrap-expmap1);
  distance = norm(expmap2-expmap1);
  distance_flip = norm(expmap2_flip-expmap1);
  if(abs(unwrap_distance-min(distance,distance_flip))>abs(unwrap_distance-max(distance,distance_flip)))
    error('unwrapExpmap is choosing the incorrect exponential map');
  end
end
end