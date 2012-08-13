function  dvcross = dcrfp( v ,x, dv, dx)
% @return d/dy crfp(v)*x
% @param v
% @param x
% @param dv = dv/dy
% @param dx = dx/dy
dvcross = [-dv(3,:)*x(2) - v(3)*dx(2,:) + dv(2,:)*x(3) + v(2)*dx(3,:);...
  -dv(1,:)*x(3) - v(1)*dx(3,:); dv(1,:)*x(2) + v(1)*dx(2,:)];