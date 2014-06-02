function [SdotV, dSdotVdq] = motionSubspaceDotV(body, qBody, vBody)
compute_gradient = nargout > 1;
nq = length(qBody);
if body.floating == 1
  % configuration parameterization: origin position in base frame and rpy
  % velocity parameterization: origin velocity in base frame and rpydot
  
  roll = qBody(4);
  pitch = qBody(5);
  yaw = qBody(6);
  
  xd = vBody(1);
  yd = vBody(2);
  zd = vBody(3);
  
  rolld = vBody(4);
  pitchd = vBody(5);
  yawd = vBody(6);
  
  cr = cos(roll);
  sr = sin(roll);
  
  cp = cos(pitch);
  sp = sin(pitch);
  
  cy = cos(yaw);
  sy = sin(yaw);
  
  SdotV = [...
    -pitchd*yawd*cp;
    rolld*yawd*cp*cr - pitchd*yawd*sp*sr - pitchd*rolld*sr;
    - pitchd*rolld*cr - pitchd*yawd*cr*sp - rolld*yawd*cp*sr;
    yd*(yawd*cp*cy - pitchd*sp*sy) - xd*(pitchd*cy*sp + yawd*cp*sy) - pitchd*zd*cp;
    zd*(rolld*cp*cr - pitchd*sp*sr) + xd*(rolld*(sr*sy + cr*cy*sp) - yawd*(cr*cy + sp*sr*sy) + pitchd*cp*cy*sr) - yd*(rolld*(cy*sr - cr*sp*sy) + yawd*(cr*sy - cy*sp*sr) - pitchd*cp*sr*sy);
    xd*(rolld*(cr*sy - cy*sp*sr) + yawd*(cy*sr - cr*sp*sy) + pitchd*cp*cr*cy) - zd*(pitchd*cr*sp + rolld*cp*sr) + yd*(yawd*(sr*sy + cr*cy*sp) - rolld*(cr*cy + sp*sr*sy) + pitchd*cp*cr*sy)];
  
  if nargout > 1
%     Generated using:
%     body.floating = 1;
%     syms roll pitch yaw rolld pitchd yawd real;
%     syms x y z xd yd zd real;
%     qBody = [x; y; z; roll; pitch; yaw];
%     vBody = [xd; yd; zd; rolld; pitchd; yawd];
%     SdotV = motionSubspaceDotV(body, qBody, vBody);
%     dSdotVdq = jacobian(SdotV(:), qBody);
%     matlabFunction(simple(dSdotVdq))
    dSdotVdq = reshape([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-pitchd*rolld*cr-pitchd*yawd*cr*sp-rolld*yawd*cp*sr,pitchd*rolld*sr+pitchd*yawd*sp*sr-rolld*yawd*cp*cr,0,xd*(rolld*(cr*sy-cy*sp*sr)+yawd*(cy*sr-cr*sp*sy)+pitchd*cp*cr*cy)-zd*(pitchd*cr*sp+rolld*cp*sr)+yd*(-rolld*(cr*cy+sp*sr*sy)+yawd*(sr*sy+cr*cy*sp)+pitchd*cp*cr*sy),-zd*(rolld*cp*cr-pitchd*sp*sr)-xd*(rolld*(sr*sy+cr*cy*sp)-yawd*(cr*cy+sp*sr*sy)+pitchd*cp*cy*sr)+yd*(rolld*(cy*sr-cr*sp*sy)+yawd*(cr*sy-cy*sp*sr)-pitchd*cp*sr*sy),pitchd*yawd*sp,-pitchd*yawd*cp*sr-rolld*yawd*cr*sp,rolld*yawd*sp*sr-pitchd*yawd*cp*cr,-xd*(pitchd*cp*cy-yawd*sp*sy)-yd*(pitchd*cp*sy+yawd*cy*sp)+pitchd*zd*sp,-zd*(pitchd*cp*sr+rolld*cr*sp)-xd*(-rolld*cp*cr*cy+pitchd*cy*sp*sr+yawd*cp*sr*sy)+yd*(rolld*cp*cr*sy+yawd*cp*cy*sr-pitchd*sp*sr*sy),-zd*(pitchd*cp*cr-rolld*sp*sr)-xd*(pitchd*cr*cy*sp+rolld*cp*cy*sr+yawd*cp*cr*sy)-yd*(-yawd*cp*cr*cy+pitchd*cr*sp*sy+rolld*cp*sr*sy),0,0,0,-xd*(yawd*cp*cy-pitchd*sp*sy)-yd*(pitchd*cy*sp+yawd*cp*sy),yd*(rolld*(sr*sy+cr*cy*sp)-yawd*(cr*cy+sp*sr*sy)+pitchd*cp*cy*sr)+xd*(rolld*(cy*sr-cr*sp*sy)+yawd*(cr*sy-cy*sp*sr)-pitchd*cp*sr*sy),yd*(rolld*(cr*sy-cy*sp*sr)+yawd*(cy*sr-cr*sp*sy)+pitchd*cp*cr*cy)-xd*(-rolld*(cr*cy+sp*sr*sy)+yawd*(sr*sy+cr*cy*sp)+pitchd*cp*cr*sy)],[6,6]);
  end
  
elseif body.floating == 2
  % configuration parameterization: origin position in base frame and quat
  % velocity parameterization: twist in body frame
  SdotV = sparse(6, 1);
  
  % previously:
%   % configuration parameterization: origin position in base frame and quat
%   % velocity parameterization: origin velocity and quatdot
%   q_s = qBody(4);
%   q_x = qBody(5);
%   q_y = qBody(6);
%   q_z = qBody(7);
%   
%   xd = vBody(1);
%   yd = vBody(2);
%   zd = vBody(3);
%   
%   q_sd = vBody(4);
%   q_xd = vBody(5);
%   q_yd = vBody(6);
%   q_zd = vBody(7);
%   
%   SdotV = [...
%     0;
%     0;
%     0;
%     (q_y^3*(2*q_xd*yd - 2*q_sd*zd) - q_z*(q_s^2*(4*q_zd*xd + 2*q_sd*yd - 2*q_xd*zd) - q_y^2*(2*q_sd*yd + 2*q_xd*zd) + q_x^2*(4*q_zd*xd - 2*q_sd*yd + 2*q_xd*zd) + q_s*(q_x*(4*q_xd*yd + 4*q_sd*zd) + q_y*(4*q_yd*yd - 4*q_zd*zd)) + q_x*q_y*(4*q_zd*yd + 4*q_yd*zd)) + q_z^3*(2*q_sd*yd + 2*q_xd*zd) + q_s^3*(2*q_zd*yd - 2*q_yd*zd) + q_x^3*(2*q_yd*yd + 2*q_zd*zd) + q_s^2*(q_y*(2*q_xd*yd - 4*q_yd*xd + 2*q_sd*zd) + q_x*(2*q_yd*yd + 2*q_zd*zd)) + q_z^2*(q_x*(4*q_xd*xd + 2*q_yd*yd - 2*q_zd*zd) - q_s*(2*q_zd*yd - 4*q_sd*xd + 2*q_yd*zd) + q_y*(2*q_xd*yd - 2*q_sd*zd)) + q_s*((2*q_zd*yd - 2*q_yd*zd)*q_x^2 + (4*q_xd*zd - 4*q_sd*yd)*q_x*q_y + (4*q_sd*xd + 2*q_zd*yd + 2*q_yd*zd)*q_y^2) - q_x^2*q_y*(4*q_yd*xd + 2*q_xd*yd + 2*q_sd*zd) + q_x*q_y^2*(4*q_xd*xd - 2*q_yd*yd + 2*q_zd*zd));
%     (q_y*(q_s^2*(2*q_xd*xd + 2*q_zd*zd) + q_x^2*(4*q_yd*yd - 2*q_xd*xd + 2*q_zd*zd) + q_z^2*(2*q_xd*xd + 4*q_yd*yd - 2*q_zd*zd) + q_z*(q_s*(4*q_yd*xd - 4*q_sd*zd) - q_x*(4*q_zd*xd + 4*q_xd*zd)) - q_s*q_x*(4*q_sd*xd + 4*q_yd*zd)) + q_x^3*(2*q_yd*xd + 2*q_sd*zd) - q_s^3*(2*q_zd*xd - 2*q_xd*zd) - q_z^3*(2*q_sd*xd - 2*q_yd*zd) + q_y^3*(2*q_xd*xd + 2*q_zd*zd) + q_z^2*(q_s*(2*q_zd*xd + 4*q_sd*yd + 2*q_xd*zd) + q_x*(2*q_yd*xd + 2*q_sd*zd)) - q_y^2*(q_x*(2*q_yd*xd + 4*q_xd*yd - 2*q_sd*zd) + q_z*(2*q_sd*xd + 4*q_zd*yd + 2*q_yd*zd) + q_s*(2*q_zd*xd - 2*q_xd*zd)) + q_z*((2*q_sd*xd - 4*q_zd*yd + 2*q_yd*zd)*q_s^2 + (4*q_xd*xd - 4*q_zd*zd)*q_s*q_x + (2*q_yd*zd - 2*q_sd*xd)*q_x^2) - q_s^2*q_x*(4*q_xd*yd - 2*q_yd*xd + 2*q_sd*zd) - q_s*q_x^2*(2*q_zd*xd - 4*q_sd*yd + 2*q_xd*zd));
%     (q_s^3*(2*q_yd*xd - 2*q_xd*yd) - q_y*(q_s^2*(2*q_sd*xd - 2*q_zd*yd + 4*q_yd*zd) - q_x^2*(2*q_sd*xd + 2*q_zd*yd) + q_z^2*(2*q_zd*yd - 2*q_sd*xd + 4*q_yd*zd) + q_s*(q_z*(4*q_zd*xd + 4*q_sd*yd) + q_x*(4*q_xd*xd - 4*q_yd*yd)) + q_x*q_z*(4*q_yd*xd + 4*q_xd*yd)) + q_x^3*(2*q_zd*xd - 2*q_sd*yd) + q_y^3*(2*q_sd*xd + 2*q_zd*yd) + q_z^3*(2*q_xd*xd + 2*q_yd*yd) + q_s^2*(q_x*(2*q_zd*xd + 2*q_sd*yd - 4*q_xd*zd) + q_z*(2*q_xd*xd + 2*q_yd*yd)) + q_y^2*(q_z*(2*q_xd*xd - 2*q_yd*yd + 4*q_zd*zd) - q_s*(2*q_yd*xd + 2*q_xd*yd - 4*q_sd*zd) + q_x*(2*q_zd*xd - 2*q_sd*yd)) + q_s*((2*q_yd*xd + 2*q_xd*yd + 4*q_sd*zd)*q_x^2 + (4*q_zd*yd - 4*q_sd*xd)*q_x*q_z + (2*q_yd*xd - 2*q_xd*yd)*q_z^2) - q_x*q_z^2*(2*q_zd*xd + 2*q_sd*yd + 4*q_xd*zd) + q_x^2*q_z*(2*q_yd*yd - 2*q_xd*xd + 4*q_zd*zd))];
  if compute_gradient
    dSdotVdq = sparse(numel(SdotV), nq);
  end
else % twist parameterized 6-DoF joint, one-DoF joints
  SdotV = sparse(6, 1);
  if compute_gradient
    dSdotVdq = sparse(numel(SdotV), nq);
  end
end

end