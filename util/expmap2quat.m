function [varargout] = expmap2quat(v) 
  varargout = cell(1, nargout);
  use_mex = (isnumeric(v) && exist('expmap2quatImpl_mex','file'));
  if use_mex
    [varargout{:}] = expmap2quatImpl_mex(v);
  else
    [varargout{:}] = expmap2quatImpl(v);
  end
end
