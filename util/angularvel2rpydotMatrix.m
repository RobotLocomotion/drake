function [Phi, dPhi, ddPhi] = angularvel2rpydotMatrix(rpy)
% Computes the matrix that transforms the angular velocity vector to the
% time derivatives of rpy (rolldot, pitchdot, yawdot).
% See eq. (5.41) in Craig05. Derivation in rpydot2angularvel.
%
% @param rpy [roll; pitch; yaw]
% @retval Phi matrix such that Phi * omega = rpyd, where omega is the
% angular velocity in world frame, and rpyd is the time derivative of
% [roll; pitch; yaw]
%
% @retval dPhi gradient of Phi with respect to rpy
% @retval ddPhi gradient of dPhi with respect to rpy

compute_gradient = nargout > 1;
compute_second_deriv = nargout > 2;

p = rpy(2);
y = rpy(3);

sy = sin(y);
cy = cos(y);
sp = sin(p);
cp = cos(p);
tp = sp / cp;

% warning: note the singularities!
Phi = ...
  [cy/cp, sy/cp, 0; ...
  -sy,       cy, 0; ...
  cy*tp, tp*sy, 1];

if compute_gradient
  sp2 = sp^2;
  cp2 = cp^2;
  dPhi = [...
    0, (cy*sp)/cp2,        -sy/cp;
    0, 0,                   -cy;
    0, cy + (cy*sp2)/cp2, -(sp*sy)/cp;
    0, (sp*sy)/cp2,         cy/cp;
    0, 0,                   -sy;
    0, sy + (sp2*sy)/cp2,  (cy*sp)/cp;
    0, 0,                    0;
    0, 0,                    0;
    0, 0,                    0];
end

if compute_second_deriv
  cp3 = cp2 * cp;
  ddPhi = [...
    0, 0,                   0;
    0, 0,                   0;
    0, 0,                   0;
    0, 0,                   0;
    0, 0,                   0;
    0, 0,                   0;
    0, 0,                   0;
    0, 0,                   0;
    0, 0,                   0;
    0, -(cy*(cp2 - 2))/cp3, (sp*sy)/(sp2 - 1);
    0, 0,                   0;
    0, (2*cy*sp)/cp3,       sy/(sp2 - 1);
    0, (2*sy - cp2*sy)/cp3, (cy*sp)/cp2;
    0, 0,                   0;
    0, (2*sp*sy)/cp3,       cy/cp2;
    0, 0,                   0;
    0, 0,                   0;
    0, 0,                   0;
    0, (sp*sy)/(sp2 - 1),   -cy/cp;
    0, 0,                   sy;
    0, sy/(sp2 - 1),        -(cy*sp)/cp;
    0, (cy*sp)/cp2,         -sy/cp;
    0, 0,                   -cy;
    0, cy/cp2,              -(sp*sy)/cp;
    0, 0,                   0;
    0, 0,                   0;
    0, 0,                   0];
    ddPhi = reshape(ddPhi, numel(Phi), []); % to match geval output
end
end