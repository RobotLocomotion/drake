function springGradientsTest()
% Tests user gradients vs numerical gradients for consistency

oldpath=addpath(fullfile(pwd,'..'));

p = RigidBodyManipulator('MassSpringDamper.urdf');
fun = @(q,qd)vectorComputeSpatialForce(p,q,qd);

% some random states to test
q = rand(1,10);
qd = rand(1,10);

for i=1:numel(q)
  f1=cell(1,2);
  [f1{:}]=geval(1,fun,q(i),qd(i),struct('grad_method','user'));

  f2=cell(1,2);
  [f2{:}]=geval(1,fun,q(i),qd(i),struct('grad_method','numerical'));

  for j=1:2
    if (any(abs(f1{j}-f2{j})>1e-5))
      path(oldpath);
      error('gradients don''t match!');
    end
  end
end

path(oldpath);

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