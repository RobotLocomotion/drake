function runAllTests(bAbortOnFail)

% Recurses through the robotlib directories, running all test scripts (in 
% test subdirectories) and all methods/scripts in the examples directory.
%   Usage:  run_all_tests([bAbortOnFail]);
%   Defaults: bAbortOnFail = true;
%
% A test function is any valid .m file which calls error() if the test
% fails, and returns without error if the test passes.  If bAbortOnFail is
% true, then the error is rethrown immediately, so that you can go in and
% debug the failure.
%
% A .m file in the test or examples directories will not be run if it has
% the string "classdef" or "NOTEST" anywhere in the file. 
%
% This script should always be run (and all tests should pass) before 
% anything is committed back into the robotlib repository.
%

if (nargin<1) bAbortOnFail = true; end

info.bAbortOnFail = bAbortOnFail;
info.passcount = 0;
info.failcount = 0;
info.initialpwd = pwd;

disp('');

pause off;
info=run_tests_in('.',info,true);
info=run_tests_in('examples',info,false);
pause on;

fprintf(1,'\n Executed %d tests.  %d passed.  %d failed.\n',info.passcount+info.failcount, info.passcount, info.failcount);

end

function info = run_tests_in(pdir,info,bOnlyLookForTestDirs)
% runs tests in a particular trajectory; gets called recursively

  p = pwd;
  cd(pdir);
  files=dir('.');
  
  for i=1:length(files)
    if (files(i).isdir)
      % then recurse into the directory
      if (files(i).name(1)~='.')  % skip . directories
        info = run_tests_in(files(i).name,info,bOnlyLookForTestDirs && ~strcmpi(files(i).name,'test'));
      end
      continue;
    end
    if (bOnlyLookForTestDirs) continue; end
    if (~strcmpi(files(i).name(end-1:end),'.m')) continue; end
    if (strcmpi(files(i).name,'Contents.m')) continue; end
    
    % check if it's a function or classdef
    if (checkFile(files(i).name,{'classdef','NOTEST'}))
      continue; 
    end
    
    % If I made it to here, then actually run the file.
    
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
        cd(info.initialpwd);
        rethrow(lasterror);
      end
    end

    % now clean up for the next guy:
    close all;
    t = timerfind;
    if (~isempty(t)) stop(t); end
    
  end 
  
  cd(p);
end

function bfound = checkFile(filename,strings)
% opens the file and checks for the existence of the string (or strings)

if ~iscell(strings), strings = {strings}; end

bfound = false;
fid=fopen(filename);
if (fid<0) return; end  % couldn't open the file.  skip it.
while true  % check the file for the string "NOTEST" (case specific)
  tline = fgetl(fid);
  if (~ischar(tline))
    break;
  end
  for i=1:length(strings)
    if (strfind(tline,strings{i}))
      fclose(fid);
      bfound = true;
      return;
    end
  end
end
fclose(fid);

end