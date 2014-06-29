function options = parseOptions(options,defaults,validate_fcns)
% sets any missing fields in the options structure to their default values
% @param options the incoming options structure
% @param defaults a structure with default values for the feilds
% @param validate_fcns is a structure with function handles which will
%   check the content of the option fields. 
% Example: 
%   functon myfun(param1, param2, options)
%
%      if nargin<3, options=struct(); end
%      defaults.option1 = true;   valid.option1 = @islogical;
%      defaults.option2 = 'blue'; valid.option2 = @(x)(ischar(x) && x(1)=='b');
%      options = parseOptions(options,defaults,valid);

fields = fieldnames(defaults);
for i=1:numel(fields)
  f=f{i};
  if ~isfield(options,f), 
    options.(f) = defaults.(f); 
  end
end

if nargin>2
  fields = fieldnames(validate_fcns);
  for i=1:numel(fields)
    assert(feval(validate_fcns.(f{i}),options.(f{i})),'Drake:InvalidOption','You have specified an invalid option');
  end
end
