function SdotV = motionSubspaceDotV(body, qBody, vBody)
if body.floating == 1 % roll, pitch, yaw
  
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
elseif body.floating == 2 % quat
  
  q_s = qBody(4);
  q_x = qBody(5);
  q_y = qBody(6);
  q_z = qBody(7);
  
  xd = vBody(1);
  yd = vBody(2);
  zd = vBody(3);
  
  q_sd = vBody(4);
  q_xd = vBody(5);
  q_yd = vBody(6);
  q_zd = vBody(7);
  
  SdotV = [...
    0;
    0;
    0;
    (q_y^3*(2*q_xd*yd - 2*q_sd*zd) - q_z*(q_s^2*(4*q_zd*xd + 2*q_sd*yd - 2*q_xd*zd) - q_y^2*(2*q_sd*yd + 2*q_xd*zd) + q_x^2*(4*q_zd*xd - 2*q_sd*yd + 2*q_xd*zd) + q_s*(q_x*(4*q_xd*yd + 4*q_sd*zd) + q_y*(4*q_yd*yd - 4*q_zd*zd)) + q_x*q_y*(4*q_zd*yd + 4*q_yd*zd)) + q_z^3*(2*q_sd*yd + 2*q_xd*zd) + q_s^3*(2*q_zd*yd - 2*q_yd*zd) + q_x^3*(2*q_yd*yd + 2*q_zd*zd) + q_s^2*(q_y*(2*q_xd*yd - 4*q_yd*xd + 2*q_sd*zd) + q_x*(2*q_yd*yd + 2*q_zd*zd)) + q_z^2*(q_x*(4*q_xd*xd + 2*q_yd*yd - 2*q_zd*zd) - q_s*(2*q_zd*yd - 4*q_sd*xd + 2*q_yd*zd) + q_y*(2*q_xd*yd - 2*q_sd*zd)) + q_s*((2*q_zd*yd - 2*q_yd*zd)*q_x^2 + (4*q_xd*zd - 4*q_sd*yd)*q_x*q_y + (4*q_sd*xd + 2*q_zd*yd + 2*q_yd*zd)*q_y^2) - q_x^2*q_y*(4*q_yd*xd + 2*q_xd*yd + 2*q_sd*zd) + q_x*q_y^2*(4*q_xd*xd - 2*q_yd*yd + 2*q_zd*zd));
    (q_y*(q_s^2*(2*q_xd*xd + 2*q_zd*zd) + q_x^2*(4*q_yd*yd - 2*q_xd*xd + 2*q_zd*zd) + q_z^2*(2*q_xd*xd + 4*q_yd*yd - 2*q_zd*zd) + q_z*(q_s*(4*q_yd*xd - 4*q_sd*zd) - q_x*(4*q_zd*xd + 4*q_xd*zd)) - q_s*q_x*(4*q_sd*xd + 4*q_yd*zd)) + q_x^3*(2*q_yd*xd + 2*q_sd*zd) - q_s^3*(2*q_zd*xd - 2*q_xd*zd) - q_z^3*(2*q_sd*xd - 2*q_yd*zd) + q_y^3*(2*q_xd*xd + 2*q_zd*zd) + q_z^2*(q_s*(2*q_zd*xd + 4*q_sd*yd + 2*q_xd*zd) + q_x*(2*q_yd*xd + 2*q_sd*zd)) - q_y^2*(q_x*(2*q_yd*xd + 4*q_xd*yd - 2*q_sd*zd) + q_z*(2*q_sd*xd + 4*q_zd*yd + 2*q_yd*zd) + q_s*(2*q_zd*xd - 2*q_xd*zd)) + q_z*((2*q_sd*xd - 4*q_zd*yd + 2*q_yd*zd)*q_s^2 + (4*q_xd*xd - 4*q_zd*zd)*q_s*q_x + (2*q_yd*zd - 2*q_sd*xd)*q_x^2) - q_s^2*q_x*(4*q_xd*yd - 2*q_yd*xd + 2*q_sd*zd) - q_s*q_x^2*(2*q_zd*xd - 4*q_sd*yd + 2*q_xd*zd));
    (q_s^3*(2*q_yd*xd - 2*q_xd*yd) - q_y*(q_s^2*(2*q_sd*xd - 2*q_zd*yd + 4*q_yd*zd) - q_x^2*(2*q_sd*xd + 2*q_zd*yd) + q_z^2*(2*q_zd*yd - 2*q_sd*xd + 4*q_yd*zd) + q_s*(q_z*(4*q_zd*xd + 4*q_sd*yd) + q_x*(4*q_xd*xd - 4*q_yd*yd)) + q_x*q_z*(4*q_yd*xd + 4*q_xd*yd)) + q_x^3*(2*q_zd*xd - 2*q_sd*yd) + q_y^3*(2*q_sd*xd + 2*q_zd*yd) + q_z^3*(2*q_xd*xd + 2*q_yd*yd) + q_s^2*(q_x*(2*q_zd*xd + 2*q_sd*yd - 4*q_xd*zd) + q_z*(2*q_xd*xd + 2*q_yd*yd)) + q_y^2*(q_z*(2*q_xd*xd - 2*q_yd*yd + 4*q_zd*zd) - q_s*(2*q_yd*xd + 2*q_xd*yd - 4*q_sd*zd) + q_x*(2*q_zd*xd - 2*q_sd*yd)) + q_s*((2*q_yd*xd + 2*q_xd*yd + 4*q_sd*zd)*q_x^2 + (4*q_zd*yd - 4*q_sd*xd)*q_x*q_z + (2*q_yd*xd - 2*q_xd*yd)*q_z^2) - q_x*q_z^2*(2*q_zd*xd + 2*q_sd*yd + 4*q_xd*zd) + q_x^2*q_z*(2*q_yd*yd - 2*q_xd*xd + 4*q_zd*zd))];
  
else % twist parameterized 6-DoF joint, one-DoF joints
  SdotV = zeros(6, 1);
end

end