function fieldcheck(s,fieldname)

if ~isfield(s,fieldname)
  error(['Missing required field ',fieldname]);
end
