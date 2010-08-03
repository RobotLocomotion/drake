function runAllTests(bAbortOnFail)

% Recurses through the robotlib directories, running all test scripts (in 
% test subdirectories).
%   Usage:  run_all_tests([bAbortOnFail]);
%   Defaults: bAbortOnFail = true;
%
% A test function is any valid .m file which calls error() if the test
% fails, and returns without error if the test passes.  If bAbortOnFail is
% true, then the error is rethrown immediately, so that you can go in and
% debug the failure.
%
% This script should always be run (and all tests should pass) before 
% anything is committed back into the robotlib repository.
%

if (nargin<1) bAbortOnFail = true; end

info.bAbortOnFail = bAbortOnFail;
info.passcount = 0;
info.failcount = 0;

disp('');

pause off;
info=run_tests_in('.',info,true);
info=run_tests_in('examples',info,false);
pause on;

fprintf(1,'\n Executed %d tests.  %d passed.  %d failed.\n',info.passcount+info.failcount, info.passcount, info.failcount);

end

function info = run_tests_in(pdir,info,bOnlyLookForTestDirs)
  p = pwd;
  cd(pdir);
  files=dir('.');
  
  for i=1:length(files)
    if (files(i).isdir)
      % then recurse into the directory
      if (files(i).name(1)~='.')  % skip . directories
        info = run_tests_in(files(i).name,info,~strcmpi(files(i).name,'test'));
      end
      continue;
    elseif bOnlyLookForTestDirs || ~strcmpi(files(i).name(end-1:end),'.m')
      % then it's not a directory or an m file.  skip it.
      continue;
    end
    testname = files(i).name;
    ind=find(testname=='.',1);
    testname=testname(1:(ind-1));

    close all;
    try
      feval(testname);
      fprintf(1,'%-40s ',testname);
      fprintf(1,'[PASSED]\n');
      info.passcount = info.passcount+1;
      
    catch
      fprintf(1,'%-40s ',testname);
      fprintf(1,'[FAILED]\n');
      info.failcount = info.failcount+1;
      if (info.bAbortOnFail)
        cd(p);
        rethrow(lasterror);
      end
    end
  end 
  
  cd(p);
end