function varargout = matrixODE(ODESOLVER,ODEFUN,tspan,A0,varargin)

% MATRIX ODE
%   simple function which abstracts the details of integrating a collection
%   of vectors and/or matrices through the ODE solvers.  Take functions of
%   the form:
%     Adot = ODEFUN(t,A)
%   where A is matrix array.  The function returns
%     tout,Aout
%   where Aout(:,:,i) is the value of A at time tout(i).

% reshape into a vector
y0 = reshape(A0,[],1);


if (nargout>1)
  % call ode solver
  [tout,yout] = ODESOLVER(@odefun_wrapper,tspan,y0,varargin{:});

  % re-arrange output
  Aout = reshape(yout',[size(A0) length(tout)]);
  varargout{1} = tout;
  varargout{2} = Aout;
else
  soln = ODESOLVER(@odefun_wrapper,tspan,y0,varargin{:});
  varargout{1} = ODESolTrajectory(soln,size(A0));
end

  % helper function
  function ydot = odefun_wrapper(t,y)
    A = reshape(y,size(A0));
    Adot = ODEFUN(t,A);
    ydot = reshape(Adot,[],1);
  end
end

