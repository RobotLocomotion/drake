function varargout = cellODE(ODESOLVER,ODEFUN,tspan,A0,varargin)

% CELL ODE
%   simple function which abstracts the details of integrating a collection
%   of vectors and/or matrices through the ODE solvers.  Take functions of
%   the form:
%     Adot = ODEFUN(t,A)
%   where A is a 1xn cell array.  The function returns
%     tout,Aout
%   where Aout{n}{i} is the value of A{i} at time tout(n).

if (~iscell(A0))
  error('A0 is not a cell array.  use normal ODE.');
end
if (prod(size(A0))~=length(A0)) 
  error('only handle cell vectors (nx1 or 1xn) so far');
end

% concat all of the vectors/matrices together:
y0=[];
for i=1:length(A0)
  y0 = [y0; reshape(A0{i},[],1)];
end

if (nargout>1)
  % call ode solver
  [tout,yout] = ODESOLVER(@odefun_wrapper,tspan,y0,varargin{:});

  % re-arrange output
  for m=1:length(tout)
    offset = 0;
    for i=1:length(A0)
      n = prod(size(A0{i}));
      Aout{m}{i} = reshape(yout(m,offset + (1:n)),size(A0{i}));
      offset = offset+n;
    end
  end
  varargout{1} = tout;
  varargout{2} = Aout;
else
  soln = ODESOLVER(@odefun_wrapper,tspan,y0,varargin{:});
  varargout{1} = ODESolTrajectory(soln,cellfun(@size,A0,'UniformOutput',false));
end

% helper function
  function ydot = odefun_wrapper(t,y)
    for i=1:length(A0)
      n = prod(size(A0{i}));
      A{i} = reshape(y(1:n),size(A0{i}));
      y = y(n+1:end);
    end
    
    Adot = ODEFUN(t,A);

    ydot = [];
    for i=1:length(A0)
      ydot = [ydot; reshape(Adot{i},[],1)];
    end
  end

end

