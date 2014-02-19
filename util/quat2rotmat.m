function M = quat2rotmat(q)

q=q/sqrt(q'*q);  % was norm
w=q(1); x=q(2); y=q(3); z=q(4);

M = [ w*w + x*x - y*y - z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y; ...
    2*x*y + 2*w*z,  w*w + y*y - x*x - z*z, 2*y*z - 2*w*x; ...
    2*x*z - 2*w*y, 2*y*z + 2*w*x, w*w + z*z - x*x - y*y ];

