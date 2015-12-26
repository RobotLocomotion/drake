function terrainInterpTest

options.floating = 'quat';
options.terrain = RigidBodyHeightMapTerrain([-7,7],[-3,3],5*[0 0; 0 1]);
%options.viewer='RigidBodyWRLVisualizer';
r = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

if (1)
%  v = RigidBodyWRLVisualizer(r.getManipulator(),options);
  v = r.constructVisualizer(options);
  x0 = [getRandomConfiguration(r.getManipulator());zeros(6,1)];
  x0(3) = 5;
  x0 = resolveConstraints(r,x0);
  v.draw(0,x0);
  [X,Y] = meshgrid(linspace(-9,9,31),linspace(-9,9,31));
  [~,normal,pos]=collisionDetectTerrain(r,[X(:),Y(:),0*X(:)]');
  valuecheck(X,reshape(pos(1,:),size(X)));
  valuecheck(Y,reshape(pos(2,:),size(Y)));
  subplot(2,1,1);
  plotTerrain(options.terrain);
  subplot(2,1,2);
  quiver3(pos(1,:),pos(2,:),pos(3,:),normal(1,:),normal(2,:),normal(3,:));
  xlabel('x');ylabel('y');zlabel('z');
  axis equal;

  tf = 10;
  traj = simulate(r,[0 tf],x0);
  v.playback(traj);
  
  xf = traj.eval(tf);
  kinsol = doKinematics(r,xf(1:getNumPositions(r)));
  [phi,~,~,~,~,~,~,~,dphidq] = contactConstraints(r,kinsol);
  phidot = dphidq*vToqdot(r.getManipulator(),xf(1:getNumPositions(r)))*xf((getNumPositions(r)+1):end);
  if (max(abs(phidot))<0.001) % then I've stopped moving
    phi = sort(phi);
    if phi(1)<-0.1 || phi(3)>0.1
      phi
      % note: only 3 because I could land part on the slant and part on the flat
      error('should have three corners on the terrain, but I don''t');
    end
  end
end


if (0)
  % extra drawing tests to make sure that I got the triangles interpolated
 % correctly...

r = setTerrain(r,5*[0 0; 1 0],[1 0 0 -.5; 0 1 0 .5; 0 0 1 0; 0 0 0 .05]);
v = r.constructVisualizer();
v.draw(0,.1*randn(6,1)+[0;0;4;0;0;0]);
Z=collisionDetect(r,[X(:),Y(:),0*X(:)]');  Z=Z(3,:);
mesh(X,Y,reshape(Z,size(X)));
xlabel('x');ylabel('y');zlabel('z');
pause;

r = setTerrain(r,5*[0 1; 0 0],[1 0 0 -.5; 0 1 0 .5; 0 0 1 0; 0 0 0 .05]);
v = r.constructVisualizer();
Z=collisionDetect(r,[X(:),Y(:),0*X(:)]');  Z=Z(3,:);
mesh(X,Y,reshape(Z,size(X)));
xlabel('x');ylabel('y');zlabel('z');
v.draw(0,.1*randn(6,1)+[0;0;4;0;0;0]);

end
