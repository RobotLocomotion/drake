function fhandle = fcompare(varargin)
% A utility for comparing multiple implementations of a given function. The typical use case might be
% to compare a Matlab and Mex implementation of a given function. 
%
% Usage: handle = fcompare(f1, f2) 
% where f1, f2, etc are function with the same signature. 
% @retval handle: a function which evaluates each of f1, f2 in turn on its arguments,
% verifies that they return the same value, and then returns the output of f1.
% 
% Usage: handle = fcompare(f1, f2, tolerances)
% @retval handle: a function which evaluates f1 and f2 in turn, and checks their
% results using the given tolerances
% @option tolerances a vector of length 1 or length(varargout) of tolerances. 
% Use nans for arguments which should not be checked. 

p = inputParser();
p.addRequired('f1', @(x) isa(x, 'function_handle'));
p.addRequired('f2', @(x) isa(x, 'function_handle'));
p.addOptional('tolerances', 1e-8, @isnumeric);

p.parse(varargin{:});
f1 = p.Results.f1;
f2 = p.Results.f2;
tolerances = p.Results.tolerances;

function varargout = handle(varargin)
  nout = nargout;
  f1_out = cell(1, nout);
  [f1_out{:}] = f1(varargin{:});
  f2_out = cell(1, nout);
  [f2_out{:}] = f2(varargin{:});
  validate(f1_out, f2_out, tolerances);
  varargout = f1_out;
end

fhandle = @handle;

end


function validate(f1_out, f2_out, tolerances)
  if length(tolerances) == 1
    tolerances = repmat(tolerances, size(f1_out));
  else
    tolerances = reshape(tolerances(1:numel(f1_out)), size(f1_out));
  end
  sizecheck(f1_out, size(tolerances));
  sizecheck(f1_out, size(f2_out));

  for j = 1:length(f1_out)
    if ~isnan(tolerances(j))
      valuecheck(f2_out{j}, f1_out{j}, tolerances(j));
    end
  end
end





