function  dvcross = dcrf( v, x, dv, dx )

% derivative of crf  spatial cross-product operator (force).
% @return d/dy crm(v)*x
% @param v
% @param x
% @param dv = dv/dy
% @param dx = dx/dy

%vcross = -[ v(3)*x(2) - v(2)*x(3) + v(6)*x(5) - v(5)*x(6);
%  -v(3)*x(1) + v(1)*x(3) - v(6)*x(4) + v(4)*x(6);
%  v(2)*x(1) - v(1)*x(2) + v(5)*x(4) - v(4)*x(5);
%  v(3)*x(5) - v(2)*x(6);
%  -v(3)*x(4) + v(1)*x(6);
%  v(2)*x(4) - v(1)*x(5) ];

dvcross = -[ dv(3,:)*x(2) - dv(2,:)*x(3) + dv(6,:)*x(5) - dv(5,:)*x(6) + v(3)*dx(2,:) - v(2)*dx(3,:) + v(6)*dx(5,:) - v(5)*dx(6,:);
  -dv(3,:)*x(1) + dv(1,:)*x(3) - dv(6,:)*x(4) + dv(4,:)*x(6) - v(3)*dx(1,:) + v(1)*dx(3,:) - v(6)*dx(4,:) + v(4)*dx(6,:);
  dv(2,:)*x(1) - dv(1,:)*x(2) + dv(5,:)*x(4) - dv(4,:)*x(5) +  v(2)*dx(1,:) - v(1)*dx(2,:) + v(5)*dx(4,:) - v(4)*dx(5,:);
  dv(3,:)*x(5) - dv(2,:)*x(6) + v(3)*dx(5,:) - v(2)*dx(6,:);
  -dv(3,:)*x(4) + dv(1,:)*x(6) - v(3)*dx(4,:) + v(1)*dx(6,:);
  dv(2,:)*x(4) - dv(1,:)*x(5) + v(2)*dx(4,:) - v(1)*dx(5,:) ];
