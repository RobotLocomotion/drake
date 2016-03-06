function numericalGradTest
  gradTest(@fun,0);
end
function [f, df] = fun(x)
  f = 1e1*x^2;
  df = 2e1*x;
end
