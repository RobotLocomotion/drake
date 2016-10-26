function testWingForceGradients

options.floating = true;
p = RigidBodyManipulator('TestWing.urdf', options);

fun = @(aoa)coeffs(p.force{1},aoa);
aoa = rand(1,10);
for i=1:size(aoa,2)
  [~,~,~,df1a,df1b,df1c]=geval(3,fun,aoa(i),struct('grad_method','user'));
  [~,~,~,df2a,df2b,df2c]=geval(3,fun,aoa(i),struct('grad_method','taylorvar'));
  if (any(any(abs(df1a-df2a)>1e-5)))
    error('gradient of lift coefficients don''t match!');
  end
  if (any(any(abs(df1b-df2b)>1e-5)))
    error('gradient of drag coefficients don''t match!');
  end
  if (any(any(abs(df1c-df2c)>1e-5)))
    error('gradient of pitch moment coefficients don''t match!');
  end
end

fun = @(q,qd)vectorComputeSpatialForce(p,q,qd);
q = [100*rand(3,20);(pi/2)*rand(3,20)];
qd = 100*rand(6,20);

for i=1:size(q,2)
  [f1,df1]=geval(1,fun,q(:,i),qd(:,i),struct('grad_method','user'));
  [f2,df2]=geval(1,fun,q(:,i),qd(:,i),struct('grad_method','taylorvar'));
  if (any(any(abs(f1-f2)>1e-5)))
    imagesc(abs(f1-f2));
    error('dynamics when computing gradients don''t match!');
  end
  if (any(any(abs(df1-df2)>1e-5)))
    imagesc(abs(df1-df2));
    error('gradients of forces don''t match!');
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