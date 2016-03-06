function h = drawEllipsoid(A_ellipsoid,b_ellipsoid,lcmgl_name)
% Draw an ellipse in lcmgl, parameterized as A_ellipsoid*x+b_ellipsoid, where x
% is a point on the unit circle
[U,S,V] = svd(A_ellipsoid);
if(det(U)*det(V)<0)
  [U,S,V] = svd(-A_ellipsoid);
end
if(det(U)<0 && det(V)<0)
  U = [U(:,2) U(:,1) U(:,3)];
  S = diag([S(2,2);S(1,1);S(3,3)]);
  V = [V(:,2) V(:,1) V(:,3)];
end
if(det(U)<0 ||det(V)<0)
  error('U and V should be both rotation matrix');
end
h = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),lcmgl_name);
h.glColor4f(0,0,1,0.5);
h.glPushMatrix();
h.glTranslated(b_ellipsoid(1),b_ellipsoid(2),b_ellipsoid(3));

U_axis = rotmat2axis(U);
h.glRotated(U_axis(4)/pi*180,U_axis(1),U_axis(2),U_axis(3));
h.glScalef(S(1,1),S(2,2),S(3,3));
V_axis = rotmat2axis(V');
h.glRotated(V_axis(4)/pi*180,V_axis(1),V_axis(2),V_axis(3));
h.sphere(zeros(3,1),1,20,20);
h.glPopMatrix();
end