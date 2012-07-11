function polyWTVCoefTest

x = PolynomialWTimeVaryingCoefficients(msspoly('x',2));
A = rand(2);

A*x;
