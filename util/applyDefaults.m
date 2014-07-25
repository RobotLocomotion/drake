function s = applyDefaults(s, defaults)
% Given a (possibly empty) struct s and a struct of default values
% defaults, return a new struct with any values missing from s filled in
% with values from defaults. The new struct will have the union of the
% fields of s and defaults.

if isempty(s)
  s = struct();
end

fields = fieldnames(defaults);
for f = fields';
  field = f{1};
  if ~isfield(s, field)
    s.(field) = defaults.(field);
  end
end