function testKinematics

m = PlanarRigidBodyModel('../Acrobot.urdf');

  function [x,J] = getPoints(m,q)  
    doKinematics(m,q);
        
    count=0;
    for i=1:length(m.body)
      body = m.body(i);
      for j=1:length(body.geometry)
        s = size(body.geometry{j}.x); n=prod(s);
        pts = [reshape(body.geometry{j}.x,1,n); reshape(body.geometry{j}.y,1,n)];
        if (nargout>1)
          [x(:,count+(1:n)),J(2*count+(1:2*n),:)] = forwardKin(body,pts);
        else
          if ~exist('x') % extra step to help taylorvar
            x = forwardKin(body,pts);
          else
            xn = forwardKin(body,pts);
            x=[x,xn];
          end
        end
        count = count + n;
      end
    end
  end

options.grad_method = {'user','taylorvar'};

for i=1:100
  q = randn(2,1); 
  [x,J] = geval(1,@getPoints,m,q,options);
end

end