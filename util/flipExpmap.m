function [w_flip,dw_flip] = flipExpmap(w)
% flip the rotation axis of exponential map, and turn the rotation angle
% from theta to 2*pi-theta. w_flip and w represent the same rotation
w_norm = norm(w);
if(w_norm>eps)
  w_dir = w/w_norm;
  w_flip = w-w_dir*2*pi;
  dw_flip = eye(3)-(w_norm^2*eye(3)-w*w')/w_norm^3*2*pi;
else
  w_flip = w;
  dw_flip = eye(3);
end
end