function terrainInterpTest


[X,Y] = meshgrid(linspace(-9,9,31),linspace(-9,9,31));
options.floating = true;
options.terrain = RigidBodyHeightMapTerrain(.2*[0 0; 0 1],[1 0 0 -.5; 0 1 0 .5; 0 0 1 0; 0 0 0 .05]);
r = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);


v = r.constructVisualizer();
x0 = .1*randn(12,1)+[5;-5;4;zeros(9,1)];
v.draw(0,x0);
[~,normal,pos]=collisionDetectTerrain(r,[X(:),Y(:),0*X(:)]');  
valuecheck(X,reshape(pos(1,:),size(X)));
valuecheck(Y,reshape(pos(2,:),size(Y)));
%mesh(reshape(pos(1,:),size(X)),reshape(pos(2,:),size(X)),reshape(pos(3,:),size(X)));
quiver3(pos(1,:),pos(2,:),pos(3,:),normal(1,:),normal(2,:),normal(3,:));
xlabel('x');ylabel('y');zlabel('z');

traj = simulate(r,[0 5],x0);
v.playback(traj);

return;

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

