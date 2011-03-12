function polysys=taylorApprox(sys,t0,x0,u0,order)

checkDependency('spot_enabled');

nX=sys.getNumStates();
nU=sys.getNumInputs();

tbar = msspoly('t',1);
xbar = msspoly('x',nX); 
ubar = msspoly('u',nU);
txubar=[tbar;xbar;ubar];

% make TaylorVars
txu=TaylorVar.init([t0;x0;u0],order);
t0=txu(1); x0=txu(1+(1:nX)); u0=txu(1+nX+(1:nU));

if (sys.getNumContStates())
  xdothat = getmsspoly(sys.dynamics(t0,x0,u0),txubar);
else
  xdothat=[];
end

if (sys.getNumDiscStates())
  xnhat = getmsspoly(sys.update(t0,x0,u0),txubar);
else
  xnhat=[];
end

if (sys.getNumOutputs())
  yhat = getmsspoly(sys.output(t0,x0,u0),txubar);
else
  yhat=[];
end

polysys = PolynomialSystem(sys.getNumContStates(),sys.getNumDiscStates(),sys.getNumInputs(),sys.getNumOutputs(),sys.isDirectFeedthrough(),sys.isTI(),xdothat,xnhat,yhat);

end

