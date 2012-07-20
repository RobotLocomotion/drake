function polyWTVCoefTest

x = PolynomialWTimeVaryingCoefficients(msspoly('x',2));
A = rand(2);

t = A*x;

sizecheck(t.getPoly(0),[2 1]);
if ~isequal(t.getPoly(0),t.getPoly(1)) 
  error('should be equal since it should be a constant trajectory');
end

c = PPTrajectory(foh([0 1],[[1;1],[2;2]]));
t=c'*x;

t.getPoly(0)
t.getPoly(1)
