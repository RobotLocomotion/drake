function [varargout] = exp2quat(v) 
  varargout = cell(1, nargout);
  use_mex = (isnumeric(v) && exist('exp2quatImpl_mex','file'));
  if use_mex
    [varargout{:}] = exp2quatImpl_mex(v);
  else
    [varargout{:}] = exp2quatImpl(v);
  end
end
