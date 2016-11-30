function errorDeprecatedFunction(replacement_name)
[ST,I] = dbstack();
name = ST(I+1).name;
throwAsCaller(MException('Drake:DeprecatedFunction',...
  '''%s'' is deprecated. Please use ''%s'' instead.', ...
  name, replacement_name));
end

