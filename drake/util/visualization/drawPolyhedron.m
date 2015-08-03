function drawPolyhedron(vertices,lcmgl_name)
if(size(vertices,1)~=3)
  error('vertices must have 3 rows');
end
[A,b,Aeq,beq] = vert2lcon(vertices');
for i = 1:size(A,1)
  face_vert = vertices(:,abs(A(i,:)*vertices-b(i))<1e-4);
  R = rotateVectorToAlign(A(i,:)',[0;0;1]);
  face_vert_rotate = R*face_vert;
  K = convhull(face_vert_rotate(1,:),face_vert_rotate(2,:));
  
  h = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,sprintf('%s%d',lcmgl_name,i));
  h.glColor4f(0,0,1,0.5);
  h.polygon(face_vert(1,K),face_vert(2,K),face_vert(3,K));
  h.switchBuffers();
  
end
end