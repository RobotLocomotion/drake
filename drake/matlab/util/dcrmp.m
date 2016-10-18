function  dvcross = dcrmp( v ,x, dv, dx)
% @return d/dy crmp(v)*x
% @param v
% @param x
% @param dv = dv/dy
% @param dx = dx/dy
n = size(dv,2);
dvcross = [zeros(1,n); dv(3,:)*x(1) + v(3)*dx(1,:) - dv(1,:)*x(3) - v(1)*dx(3,:);...
  -dv(2,:)*x(1) - v(2)*dx(1,:) + dv(1,:)*x(2) + v(1)*dx(2,:)];
end

