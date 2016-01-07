function param_sets = getDefaults(r)
% NOTEST
  typecheck(r, 'Valkyrie');
  param_sets = struct('walking', valkyrieParams.Walking(r),...
                      'standing', valkyrieParams.Standing(r),...
                      'position_control', valkyrieParams.PositionControl(r),...
                      'recovery', valkyrieParams.Recovery(r),...
                      'manip',valkyrieParams.Manip(r));
end
