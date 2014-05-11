function testWingForceGradients
%{
length(q) = 5, 10 total states.
[x pos
 z pos
 Theta (negative = pitch up
 Elevator angle
 Leg angle]
%}

options.floating = true;
p = RigidBodyManipulator('TestWing.urdf', options);

% some random states to test the force gradients
fun = @(q,qd)vectorComputeSpatialForce(p,q,qd);
q = [100*rand(3,10);1.5*rand(3,10)];
qd = 100*rand(6,10);
for i=1:size(q,2)
  [f1,df1]=geval(1,fun,q(:,i),qd(:,i),struct('grad_method','user'));
  [f2,df2]=geval(1,fun,q(:,i),qd(:,i),struct('grad_method','numerical'));
  if (any(abs(f1-f2)>1e-5))
    error('dynamics when computing gradients don''t match!');
  end
  if (any(abs(df1-df2)>1e-5))
    error('gradients don''t match!');
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