function s = applyDefaults(s, defaults, validate_fcns)
% Given a (possibly empty) struct s and a struct of default values
% defaults, return a new struct with any values missing from s filled in
% with values from defaults. The new struct will have the union of the
% fields of s and defaults.
% @param s the incoming  structure
% @param defaults a structure with default values for the fields
% @param validate_fcns is a structure with function handles which will
%   check the content of the fields.
% Example:
%   functon myfun(param1, param2, options)
%
%      if nargin<3, options=struct(); end
%      defaults.option1 = true;   valid.option1 = @islogical;
%      defaults.option2 = 'blue'; valid.option2 = @(x)(ischar(x) && x(1)=='b');
%      options = applyDefaults(options,defaults,valid);

if isempty(s)
  s = struct();
end

fields = fieldnames(defaults);
for i=1:numel(fields)
  f = fields{i};
  if ~isfield(s, f)
    s.(f) = defaults.(f);
  end
end

if nargin>2
  fields = fieldnames(validate_fcns);
  for i=1:numel(fields)
    f = fields{i};
    assert(feval(validate_fcns.(f),s.(f)),'Drake:InvalidOption','You have specified an invalid option');
  end
end
