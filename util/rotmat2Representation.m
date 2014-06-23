function [qrot, dqrot] = rotmat2Representation(rotation_type, R, dR)
% Front-end for the various rotmat2* functions

compute_gradient = nargout > 1;

switch rotation_type
  case 0
    qrot = zeros(0, 1);
    if compute_gradient
      dqrot = zeros(numel(qrot), size(dR, 2));
    end
  case 1
    if compute_gradient
      [qrot, dqrot] = rotmat2rpy(R, dR);
    else
      qrot = rotmat2rpy(R);
    end
  case  2
    if compute_gradient
      [qrot, dqrot] = rotmat2quat(R, dR);
    else
      qrot = rotmat2quat(R);
    end
  otherwise
    error('rotation type not recognized');
end
end