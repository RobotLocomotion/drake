function runAllTests(numToSkip,bAbortOnFail)

% Recurses through the robotlib directories, running all test scripts 
% (in test subdirectories) and all methods/scripts in the examples directory.
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

if (nargin<1) numToSkip = 0; end
if (nargin<2) bAbortOnFail = true; end

info.bAbortOnFail = bAbortOnFail;
info.numToSkip = numToSkip;
info.passcount = 0;
info.failcount = 0;
info.initialpwd = pwd;

disp('');

%pause off;
info=run_tests_in('examples',info,false);
info=run_tests_in('.',info,true);
%pause on;

clear robotlib_unit_test;
fprintf(1,'\n Executed %d tests.  %d passed.  %d failed.\n',info.passcount+info.failcount, info.passcount, info.failcount);

end

function info = run_tests_in(pdir,info,bOnlyLookForTestDirs)
% runs tests in a particular trajectory; gets called recursively

  p = pwd;
  cd(pdir);
  disp([pdir,'/']);drawnow;
  files=dir('.');
  
  for i=1:length(files)
    if (files(i).isdir)
      % then recurse into the directory
      if (files(i).name(1)~='.' && ~any(strcmpi(files(i).name,{'dev','slprj'})))  % skip . and dev directories
        info = run_tests_in(files(i).name,info,bOnlyLookForTestDirs && ~strcmpi(files(i).name,'test'));
      end
      continue;
    end
    if (bOnlyLookForTestDirs) continue; end
    if (~strcmpi(files(i).name(end-1:end),'.m')) continue; end
    if (strcmpi(files(i).name,'Contents.m')) continue; end
    
    % reject if there is a notest
    if (checkFile(files(i).name,'NOTEST'))
      continue; 
    end
    
    testname = files(i).name;
    ind=find(testname=='.',1);
    testname=testname(1:(ind-1));

    % if it's a class, see if it implements the static run method()
    isClass = checkFile(files(i).name,'classdef');
    if (isClass && ~ismethod(testname,'run'))
      continue;  % skip classes that aren't runnable
    end
    
    if (info.numToSkip>info.passcount)
      fprintf(1,'%-40s ',testname);
      fprintf(1,'[SKIPPED]\n');
      info.passcount=info.passcount+1;
      continue;
    end
    
    % If I made it to here, then actually run the file.

    force_close_system();
    close all;
    
    attemptsleft=1;
    if (checkFile(files(i).name,'OKTOFAIL'));
      attemptsleft=3;
    end
    
    while (attemptsleft)
      attemptsleft=attemptsleft-1;
      try
        if (isClass) feval([testname,'.run']);
        else feval(testname); end
        fprintf(1,'%-40s ',testname);
        fprintf(1,'[PASSED]\n');
        info.passcount = info.passcount+1;
        attemptsleft=0;
      catch
        fprintf(1,'%-40s ',testname);
        if (attemptsleft>0)
          fprintf(1,'[RETRY]\n');
        else
          fprintf(1,'[FAILED]\n');
          info.failcount = info.failcount+1;
          disp(['Run runAllTests(',num2str(info.passcount),') to continue where you left off']);
          if (info.bAbortOnFail)
            cd(info.initialpwd);
            rethrow(lasterror);
          end
        end
      end
    end
    
    % now clean up for the next guy:
    force_close_system;
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
