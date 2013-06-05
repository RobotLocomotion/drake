function debugMexTester

% run this method, then run ./debugMex debugMexTest at the linux command
% line

for i=1:10
  debugMexEval('debugMexTest',i,char('a'+i-1));
end