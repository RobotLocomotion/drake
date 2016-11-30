function msg = populateLCMFields(msg, data, defaultval)
% We often have a MATLAB struct of parameters that we would like to copy over to an
% equivalent LCM message. Rather than having to copy each field individually, we
% can just call this function to populate every field in the LCM messsage msg
% with the element of the exact same name in the MATLAB struct data.
% @param msg an LCM message object
% @param data a MATLAB struct with field names matching the fields of msg
% @param defaultval the default to use for any entries missing from data
% @retval msg the LCM message with all fields populated

if ~isstruct(data) && ~isempty(data)
  % Not a structure, so just copy it entirely
  msg = data;
  return
end

if nargin < 3
  defaultval = 0;
end

cls = msg.getClass();
fields = cls.getFields();
for j = 1:length(fields)
  field_name = char(fields(j).getName());
  if (strcmp(field_name, 'LCM_FINGERPRINT') || strcmp(field_name, 'LCM_FINGERPRINT_BASE'))
    continue;
  end
  if strfind(char(java.lang.reflect.Modifier.toString(fields(j).getModifiers())), 'final')
    % Don't try to overwrite constant ('final') fields
    continue;
  end
  if isfield(data, field_name)
    if isstruct(data.(field_name))
      msg.(field_name) = populateLCMFields(msg.(field_name), data.(field_name), defaultval);
    else
      msg.(field_name) = data.(field_name);
    end
  else
    msg.(field_name) = defaultval;
  end
end

