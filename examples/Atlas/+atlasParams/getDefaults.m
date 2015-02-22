function param_sets = getDefaults(r)
  typecheck(r, 'Atlas');
  param_sets = struct('walking', atlasParams.Walking(r),...
                      'standing', atlasParams.Standing(r));
end
