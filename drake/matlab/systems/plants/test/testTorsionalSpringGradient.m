function testTorsionalSpringGradient 

p = RigidBodyManipulator('TorsionalSpring.urdf');
fun = @(q,qd)vectorComputeSpatialForce(p,q,qd);

% some random states to test
q = 100*rand(1,10);
qd = 100*rand(1,10);

for i=1:size(q,2)
  f1=cell(1,2);
  [f1{:}]=geval(1,fun,q(i),qd(i),struct('grad_method','user'));

  f2=cell(1,2);
  [f2{:}]=geval(1,fun,q(i),qd(i),struct('grad_method','numerical'));
  
  if (any(any(abs(f1{1}-f2{1}))>1e-5))
    error('forces when computing gradients don''t match!');
  end
  if (any(any(abs(f1{2}-f2{2})>1e-5)))
    error('computing gradients don''t match!');
  end
  
end

end

function [f_ext,df_ext] = vectorComputeSpatialForce(p,q,qd)
if (nargout>1)
    [f_ext,df_ext] = p.force{1}.computeSpatialForce(p,q,qd);
    f_ext = reshape(f_ext,numel(f_ext),1);
    df_ext = reshape(df_ext,numel(f_ext),size(q,1)+size(qd,1));
else
    f_ext = p.force{1}.computeSpatialForce(p,q,qd);
    f_ext = reshape(f_ext,numel(f_ext),1);
end
end
