function polysys=taylorApprox(sys,t0,x0,u0,order)

tbar = msspoly('t',1);
xbar = msspoly('x',sys.getNumStates()); 
ubar = msspoly('u',sys.getNumInputs());

if (sys.getNumContStates())
  xdot0 = sys.dynamics(t0,x0,u0);
  dxdot = sys.dynamicsGradients(t0,x0,u0,order);
  xdothat = xdot0;
  for i=1:order % Contract the tensors.
    xdothat = xdothat + contract_tensor(dxdot{i},[tbar;xbar;ubar])/factorial(i);
  end
else
  xdothat=[];
end

if (sys.getNumDiscStates())
  xn0 = sys.update(t0,x0,u0)
  dxn = sys.updateGradients(t0,x0,u0,order);
  xnhat = xn0;
  for i=1:order
    xnhat = xnhat + contract_tensor(dxn{i},[tbar;xbar;ubar])/factorial(i);
  end
else
  xnhat=[];
end

if (sys.getNumOutputs())
  y0 = sys.output(t0,x0,u0);
  dy = sys.outputGradients(t0,x0,u0,order);
  yhat = y0;
  for i=1:order
    yhat = yhat + contract_tensor(dy{i},[tbar;xbar;ubar])/factorial(i);
  end
else
  yhat=[];
end

polysys = PolynomialSystem(sys.getNumContStates(),sys.getNumDiscStates(),sys.getNumInputs(),sys.getNumOutputs(),sys.isDirectFeedthrough(),sys.isTI(),xdothat,xnhat,yhat);

end


function o = contract_tensor(T,x)

if isnumeric(T)
  o = T*x;
else
  s = [];
  for j = 1:length(T)
    s = [s contract_tensor(T{j},x)];
  end
  o = s*x;
end

end