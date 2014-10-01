function terrainInterpTest


options.floating = true;
options.terrain = RigidBodyHeightMapTerrain([-7,7],[-3,3],5*[0 0; 0 1]);
%options.viewer='RigidBodyWRLVisualizer';
r = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

if (1)
  v = r.constructVisualizer(options);
  x0 = .1*rand(12,1)+[6*rand(2,1)-3;4;zeros(9,1)];
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

  traj = simulate(r,[0 5],x0);
  v.playback(traj);
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
