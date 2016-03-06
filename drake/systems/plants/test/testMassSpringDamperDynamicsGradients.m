function testMassSpringDamperDynamicsGradients()
% Tests user gradients vs numerical gradients for consistency

p = RigidBodyManipulator('MassSpringDamper.urdf');

for pi=1:2
  if pi==2, 
    f = p.force{1};
    f.b=0;
    p = setForce(p,1,f);
    p = compile(p);
  end
  fun = @(q,qd)vectorManipulatorDynamics(p,q,qd);
  
  % some random states to test
  q = 100*rand(1,10);
  qd = 100*rand(1,10);
  
  for i=1:size(q,2)
    f1=cell(1,6);
    [f1{:}]=geval(3,fun,q(i),qd(i),struct('grad_method','user'));
    
    f2=cell(1,6);
    [f2{:}]=geval(3,fun,q(i),qd(i),struct('grad_method','numerical'));
    
    for j=1:3
      if (any(abs(f1{j}-f2{j})>1e-5))
        error('dynamics when computing gradients don''t match!');
      end
    end
    for j=4:6
      if (any(abs(f1{j}-f2{j})>1e-5))
        error('gradients don''t match!');
      end
    end
  end
end

end

function [H,C,B,dH,dC,dB] = vectorManipulatorDynamics(p,q,qd)
if (nargout>1)
  [H,C,B,dH,dC,dB] = manipulatorDynamics(p,q,qd);
  H = reshape(H,numel(H),1);
  C = reshape(C,numel(C),1);
  B = reshape(B,numel(B),1);
  dH = reshape(dH,numel(H),size(q,1)+size(qd,1));
  dC = reshape(dC,numel(C),size(q,1)+size(qd,1));
  dB = reshape(dB,numel(B),size(q,1)+size(qd,1));
else  
  [H,C,B] = manipulatorDynamics(p,q,qd);
  H = reshape(H,numel(H),1);
  C = reshape(C,numel(C),1);
  B = reshape(B,numel(B),1);
end
end