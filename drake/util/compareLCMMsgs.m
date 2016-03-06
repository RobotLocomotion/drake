function varargout = compareLCMMsgs(msg1, msg2)
% Recursively compare all fields in a pair of LCM messages, for example to debug
% a new LCM encode method. If an output is requested, it returns whether the
% messages are identical as a boolean. If no output is requested, it raises
% an error if !ok.
% 
% Why is this useful? For example, if your LCM type describes a matrix of size 2x2:
% 
% struct my_type {
%   double my_field[2][2]
% }
% 
% you can still construct an instance of that message in matlab with the wrong size:
% >> m = my_type(); m.my_field = zeros(4,4);
% >> m.my_field
% ans =
%
%      0     0     0     0
%      0     0     0     0
%      0     0     0     0
%      0     0     0     0
% 
% But if you attempt to send and receive that message (or just copy it), your data will
% be silently truncated:
% >> m.copy().my_field
% ans =

%      0     0
%      0     0
%
% That's bad. This function will help you catch errors of that type:
% >> compareLCMMsgs(m, m.copy())
% Error using compareLCMMsgs (line 45)
% Messages do not match at field name my_field in LCM class drake.my_type.

varargout = cell(1, nargout);

cls = msg1.getClass();
fields = cls.getFields();
for j = 1:length(fields)
  field_name = char(fields(j).getName());
  if (strcmp(field_name, 'LCM_FINGERPRINT') || strcmp(field_name, 'LCM_FINGERPRINT_BASE'))
    continue;
  end
  if isjava(msg1.(field_name))
    [varargout{:}] = compareLCMMsgs(msg1.(field_name), msg2.(field_name));
  else
    ok = valuecheck(msg1.(field_name), msg2.(field_name));
    if ~ok && nargout == 0
      msg1.(field_name)
      msg2.(field_name)
      error('Drake:LCMMessagesDoNotMatch', 'Messages do not match at field name %s in LCM %s.', field_name, char(cls.toString()));
    elseif ~ok
      varargout = {ok};
      return
    end
  end
end
