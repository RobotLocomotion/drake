function debugMexTester

% run this method, then run drake/bin/debugMex debugMexTest at the linux command
% line (note: I recommend adding drake/bin to your LD_LIBRARY_PATH)

doubleIntegrator = LinearSystem([0 1; 0 0],[0;1],[],[],[1 0],0)

for i=1:10
  debugMexEval('debugMexTest',i,char('a'+i-1),doubleIntegrator);
end