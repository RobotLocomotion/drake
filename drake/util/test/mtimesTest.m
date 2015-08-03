function mtimesTest
% simple test which revealed that the mtimes for taylorvar*nontaylorvar was
% broken (and probably never worked!).

x = TaylorVar.init([2; 1], 3);

[10,11]*x

x'*[10; 11]
