function runAllTests(numToSkip,options)

% Recurses through the Drake directories, running all test scripts 
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
% anything is committed back into the Drake repository.
%

if (nargin<1) numToSkip = 0; end
if (nargin<2) options = struct(); end

if ~isfield(options,'abort_on_fail') options.abort_on_fail = false; end

info.bAbortOnFail = options.abort_on_fail;
info.numToSkip = numToSkip;
info.passcount = 0;
info.failcount = 0;
info.initialpwd = pwd;

disp('');

if (options.show_gui)
  figure(301);
  info.gui_parent = uitreenode('v0','root','All Tests',[],false);
  [tree,control] = uitree('v0','Root',info.gui_parent);
end

%pause off;
info=run_tests_in('examples',info,false);
info=run_tests_in('.',info,true);
%pause on;

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
    
    testname = files(i).name;
    ind=find(testname=='.',1);
    testname=testname(1:(ind-1));

    % if it's a class, check if it is "runnable"
    % todo: run any public, static, non-hidden, non-abstract methods?
    isClass = checkFile(files(i).name,'classdef');
    if (isClass)
      if (checkClass(files(i).name,'NOTEST'))
        continue; 
      end
      
    else
      % reject if there is a notest
      if (checkFile(files(i).name,'NOTEST'))
        continue;
      end
    end
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

    close all;
    megaclear

    s=dbstatus;
    if (any(strcmp('error',{s.cond})))  % when 'dbstop if error' is on, then run without try catch (for debugging) 
      if (isClass) feval([testname,'.run']);
      else feval(testname); end
      fprintf(1,'%-40s ',testname);
      fprintf(1,'[PASSED]\n');
      info.passcount = info.passcount+1;
    else
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
    end
    
    a=warning;
    if (~strcmp(a(1).state,'on'))
      error('somebody turned off warnings on me!');  % see bug 
    end
    
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
while true  % check the file for the strings
  tline = fgetl(fid);
  if (~ischar(tline))
    break;
  end
  for i=1:length(strings)
    if (~isempty(strfind(tline,strings{i})))
      fclose(fid);
      bfound = true;
      return;
    end
  end
end
fclose(fid);

end

function bfound = checkClass(filename,strings)

if ~iscell(strings), strings = {strings}; end
strings = lower(strings);

bfound = false;
bInMethod = false;
endcount = 0;
fid=fopen(filename);
if (fid<0) return; end  % couldn't open the file.  skip it.
while true  % check the file for the strings
  tline = fgetl(fid);
  if (~ischar(tline))
    break;
  end
  tline = lower(tline);
  commentind = strfind(tline,'%');
  if (~isempty(commentind)) tline = tline(1:commentind(1)-1); end
  if (~bInMethod && ~isempty(strfind(tline,'function')))
    bInMethod = true;
    endcount=0;
  end
  if (bInMethod)
    strings={'for','while','switch','try','if'};
    endcount = endcount + length(keywordfind(tline,strings));
    endcount = endcount - length(keywordfind(tline,'end'));
%    disp([num2str(endcount,'%2d'),': ',tline]);
    if endcount<0
      bInMethod=false;
    end
  end
end
fclose(fid);

end

function bfound = checkMethod(class,methodname,strings)

if ~iscell(strings), strings = {strings}; end
strings = lower(strings);

bfound = false;
bInMethod = false;
endcount = 0;
fid=fopen(filename);
if (fid<0) return; end  % couldn't open the file.  skip it.
while true  % check the file for the strings
  tline = fgetl(fid);
  if (~ischar(tline))
    break;
  end
  tline = lower(tline);
  commentind = strfind(tline,'%');
  if (~isempty(commentind)) tline = tline(1:commentind(1)-1); end
  if (~bInMethod && ~isempty(strfind(tline,'function')))
    if (~isempty(strfind(tline,lower(methodname))))
      bInMethod = true;
      endcount=0;
    end
  end
  if (bInMethod)
    strings={'for','while','switch','try','if'};
    endcount = endcount + length(keywordfind(tline,strings));
    endcount = endcount - length(keywordfind(tline,'end'));
    
    for i=1:length(strings)
      if (~isempty(strfind(tline,strings{i})))
        fclose(fid);
        bfound = true;
        return;
      end
    end
%    disp([num2str(endcount,'%2d'),': ',tline]);
    if endcount<0
      fclose(fid);
      return;
    end
  end
end
fclose(fid);

end


function inds = keywordfind(line,strs)

if (~iscell(strs)) strs={strs}; end

inds=[];
for i=1:length(strs)
  s = strs{i};
  a = strfind(line,s);
  % check that it is bracketed by a non-letter
  j=1;
  while j<=length(a)
    if (a(j)~=1 && isletter(line(a(j)-1)))
      a(j)=[];
      continue;
    end
    if (a(j)+length(s)<=length(line) && isletter(line(a(j)+length(s))))
      a(j)=[];
      continue;
    end
    inds=[inds,a(j)];
    j=j+1;
  end
end  

end
