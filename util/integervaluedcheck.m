function [tf,errstr]=integervaluedcheck(x)

% check if x has an integer value.
%  true for any numeric type with mod(x,1)==0

tf = true;
if ~isnumeric(x)
  tf = false;
  errstr = 'Expected a numeric value';
elseif any(mod(x,1)~=0)
  tf = false;
  errstr = 'Expected an integer-valued variable';
end

if ~tf && nargout<1
  error('Drake:IntegerValuedCheck',errstr);
end