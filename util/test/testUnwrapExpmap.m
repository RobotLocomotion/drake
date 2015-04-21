function testUnwrapExpmap()
for i = 1:100
  quat1 = uniformlyRandomQuat();
  quat2 = uniformlyRandomQuat();
  expmap1 = quat2expmap(quat1);
  expmap2 = quat2expmap(quat2);
  expmap2_unwrap = unwrapExpmap(expmap1,expmap2);
  expmap2_flip = flipExpmap(expmap2);
  if(norm(expmap2_unwrap-expmap1)>min(norm(expmap2_flip-expmap1),norm(expmap2-expmap1))+eps)
    error('unwrapExpmap is choosing the incorrect exponential map');
  end
end
end