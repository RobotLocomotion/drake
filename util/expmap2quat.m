function [varargout] = expmap2quat(v, use_mex) 
  if nargin < 2, use_mex = true; end
  varargout = cell(1, max(nargout,1));
  use_mex = use_mex && isnumeric(v);
  if use_mex
    [varargout{:}] = expmap2quatImplmex(v);
  else
    [varargout{:}] = expmap2quatImpl(v);
  end
  if nargout > 2
    % convert to strange second derivative output format
    varargout{3} = reshape(varargout{3}, size(varargout{2}, 1), []); 
  end
end
