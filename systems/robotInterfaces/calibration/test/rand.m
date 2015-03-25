function varargout = rand(varargin)
if nargin == 0
  varargout{1} = 0.1;
elseif nargin == 1
  varargout{1} = 0.1 * ones(varargin{1});
elseif nargin == 2
  varargout{1} = 0.1 * ones(varargin{1}, varargin{2});
else
  error('bla');
end
end