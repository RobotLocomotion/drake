function testBrickKinematics

options.floating = true;
m = RigidBodyModel('FallingBrick.urdf',options);
v = RigidBodyWRLVisualizer(CoordinateFrame('brickState',12,'x'),m);

  function [x,J] = getPoints(m,q)  
    doKinematics(m,q);
        
    count=0;
    for i=1:length(m.body)
      body = m.body(i);
      n = size(body.contact_pts,2);
      if (n>0)
        if (nargout>1)
          [x(:,count+(1:n)),J(3*count+(1:3*n),:)] = forwardKin(body,body.contact_pts);
        else
          if ~exist('x') % extra step to help taylorvar
            x = forwardKin(body,body.contact_pts);
          else
            xn = forwardKin(body,body.contact_pts);
            x=[x,xn];
          end
        end
        count = count + n;
      end
    end
  end

options.grad_method = {'user','taylorvar'};

for i=1:100
  q = randn(6,1); 
  [x,J] = geval(1,@getPoints,m,q,options);
end

end