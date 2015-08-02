function uTest_GetFullPath(doSpeed)  %#ok<INUSD>
% Automatic test: GetFullPath
% This is a routine for automatic testing. It is not needed for processing and
% can be deleted or moved to a folder, where it does not bother.
%
% uTest_GetFullPath(doSpeed)
% INPUT:
%   doSpeed: Optional logical flag to trigger time consuming speed tests.
%            Default: TRUE. If no speed test is defined, this is ignored.
% OUTPUT:
%   On failure the test stops with an error.
%
% Tested: Matlab 6.5, 7.7, 7.8, 7.13, WinXP/32, Win7/64
% Author: Jan Simon, Heidelberg, (C) 2009-2012 matlab.THISYEAR(a)nMINUSsimon.de

% $JRev: R-r V:039 Sum:IH0Cf8UBV1d9 Date:15-Jan-2013 01:06:12 $
% $License: BSD $
% $File: Tools\UnitTests_\uTest_GetFullPath.m $
% History:
% 024: 25-Oct-2009 16:48, BUGFIX: Tests of rejected bad input were too lazy.
% 028: 21-Jul-2010 10:48, Test UNC paths.
% 031: 24-Jan-2011 11:20, Cell string input.
% 035: 09-Aug-2012 14:00, Test paths starting with "//", which are not UNC.
%      The former version treated "\\?\C:\<longpath>\file" as UNC path and
%      replied "\\?\UNC\?\C:\<longpath>\file". Fixed now.
% 039: 08-Jan-2013 07:54, 'fat', 'lean', 'auto' type.

% Initialize: ==================================================================
FuncName  = mfilename;
FSep      = filesep;
whichFunc = which('GetFullPath');
ErrID     = ['JSimon:', FuncName, ':Failed'];
isWindows = ispc;

[dum, fName, fExt] = fileparts(whichFunc);  %#ok<ASGLU>
examined           = [fName, fExt];
   
if isWindows
   magicLong = '\\?\';
   magicUNC  = '\\?\UNC\';
else  % No magic strings for Linux:
   magicLong = '';
   magicUNC  = '';
end

% Global Interface: ------------------------------------------------------------
% Do the work: =================================================================
disp(['==== Test GetFullPath  ', datestr(now, 0), ...
   char(10), '  Function: ', whichFunc]);

% Restore folder on error - onCleanup since Matlab 7.6:
backDir = cd;
if [100, 1] * sscanf(version, '%d.', 2) >= 706
   Cleanup(backDir);
   restoreFolder = onCleanup(@Cleanup);
end

% Change to a reproducible location:
cd(prefdir);
% Alternatives:
% cd(fileparts(mfilename('fullpath')));
% cd(tempdir);
% cd('\\server\share');

cd_ = cd;
fprintf('  Working path: %s\n', cd_);

% Known answer tests: ----------------------------------------------------------
fprintf('\n== Known answer tests:\n');

longName    = repmat('A', 1, 262);
longReplace = sprintf('<long:%d chars>', length(longName));

if isWindows
   Base = 'D:\Temp\Folder';  % Need not exist!
else
   Base = '/Temp/Folder';
end

TestSet = { ...
   ... % Input:             Output:
   '',                      cd_; ...
   ['Folder', FSep],        fullfile(cd_, 'Folder', FSep); ...
   'File',                  fullfile(cd_, 'File'); ...
   '.',                     cd_; ...
   ['.', FSep, '.'],        cd_; ...
   '*',                     fullfile(cd_, '*'); ...
   fullfile(cd_, '.'),      cd_; ...
   fullfile(cd_, longName),       fullfile(magicLong, cd_, longName); ...
   fullfile(cd_, '.', longName),  fullfile(magicLong, cd_, longName); ...
   fullfile(cd_, longName, '..'),                 cd_; ...
   fullfile(cd_, longName, '..', longName, '..'), cd_; ...
   fullfile(cd_, longName, longName), ...             % Input
   fullfile(magicLong, cd_, longName, longName); ...  % Want
   Base,                             Base; ...
   fullfile(Base, 'Sub'),            fullfile(Base, 'Sub'); ...
   fullfile(Base, 'Sub', filesep),   fullfile(Base, 'Sub', filesep); ...
   fullfile(Base, '.'),              Base; ...
   fullfile(Base, '.', '.'),         Base; ...
   fullfile(Base, 'X', '..'),        Base; ...
   fullfile(Base, 'X', '..', 'Sub'), fullfile(Base, 'Sub'); ...
   fullfile(Base, 'X', '..', longName), fullfile(magicLong, Base, longName)};

if isWindows
   % UNC paths under Windows only!
   UNCPath = '\\Server\Folder';
   UNCBare = UNCPath(3:end);

   TestSet = cat(1, TestSet, { ...
      fullfile(Base, '..', '..'),               'D:\'; ...
      fullfile(Base, '..', '..', '..'),         'D:\'; ...
      fullfile(Base, '..', '.', '..'),          'D:\'; ...
      fullfile(Base, '.', '..', '..'),          'D:\'; ...
      fullfile('D:\', '..', 'Sub1'),            fullfile('D:\', 'Sub1'); ...
      fullfile('D:\', '..', '.', 'Sub1'),       fullfile('D:\', 'Sub1'); ...
      fullfile('D:\', '..', '.', 'Sub1', '..'), 'D:\'; ...
      fullfile('D:\', 'Sub1', '..', 'Sub2'),    fullfile('D:\', 'Sub2'); ...
      ... % UNC:
      UNCPath,                              UNCPath; ...
      fullfile(UNCPath, 'Sub'),             fullfile(UNCPath, 'Sub'); ...
      fullfile(UNCPath, 'Sub', '.'),        fullfile(UNCPath, 'Sub'); ...
      fullfile(UNCPath, 'Sub', '..'),       UNCPath; ...
      fullfile(UNCPath, 'Sub', '..', '..'), UNCPath; ...
      fullfile(UNCPath, '..', '..'),        UNCPath; ...
      fullfile(UNCPath, '.', '..'),         UNCPath; ...
      fullfile(UNCPath, '.', 'Sub', '..'),  UNCPath; ...
      fullfile(UNCPath, FSep),              fullfile(UNCPath, FSep); ...
      fullfile(UNCPath, 'Sub1', 'Sub2', '..', '..'), UNCPath; ...
      fullfile(UNCPath, '..', 'Sub1'),      fullfile(UNCPath, 'Sub1'); ...
      fullfile(UNCPath, longName, '..'),    UNCPath; ...
      fullfile(UNCPath, longName, '..', longName, '..'), UNCPath; ...
      fullfile(UNCPath, longName, '.'),     ...             % Input
      fullfile(magicUNC, UNCBare, longName); ...            % Want
      fullfile(UNCPath, longName, longName), ...            % Input
      fullfile(magicUNC, UNCBare, longName, longName); ...  % Want
      });
   
else
   TestSet = cat(1, TestSet, { ...
      fullfile(Base, '..', '..'),             '/'; ...
      fullfile(Base, '..', '..', '..'),       '/'; ...
      fullfile(Base, '..', '.', '..'),        '/'; ...
      fullfile(Base, '.', '..', '..'),        '/'; ...
      fullfile('/', '..', 'Sub1'),            fullfile('/', 'Sub1'); ...
      fullfile('/', '..', '.', 'Sub1'),       fullfile('/', 'Sub1'); ...
      fullfile('/', '..', '.', 'Sub1', '..'), '/'; ...
      fullfile('/', 'Sub1', '..', 'Sub2'),    fullfile('/', 'Sub2'); ...
      });
end

% Run through test suite string by string:
for iTest = 1:size(TestSet, 1)
   In   = TestSet{iTest, 1};
   Want = TestSet{iTest, 2};
   Got  = GetFullPath(In);

   In_   = strrep(In,   longName, longReplace);
   Got_  = strrep(Got,  longName, longReplace);
   Want_ = strrep(Want, longName, longReplace);
   
   if strcmpi(Got, Want)
      fprintf('  ok:  [%s]\n    => [%s]\n', In_, Want_);
   else
      error(ErrID, 'Failed!\nInput:  [%s]\nWanted: [%s]\nOutput: [%s]', ...
         In_, Want_, Got_);
   end
end

% Check cell string: -----------------------------------------------------------
Got = GetFullPath(TestSet(:, 1));
if isequal(Got, TestSet(:, 2))
   disp('  ok:  Cell string');
else
   error(ErrID, 'Failed for cell string.');
end

fullpath = GetFullPath({});
if isa(fullpath, 'cell') && isempty(fullpath)
   disp('  ok:  {}');
else
   error(ErrID, [FuncName, ': GetFullPath failed for {}']);
end

fullpath = GetFullPath(cell(1, 1));  % NULL pointer!
if isa(fullpath, 'cell') && length(fullpath) == 1 && strcmpi(fullpath{1}, cd_)
   disp('  ok:  {NULL}');
else
   error(ErrID, [FuncName, ': GetFullPath failed for {NULL}']);
end

% Realtive to long path: -------------------------------------------------------
fprintf('\n== Relative to PREFDIR:\n');

% Separate current directory into cell:
pd_      = prefdir;  % cd_ could be too short as "D:\Temp"
cd(pd_);
fl       = Str2Cell_L(pd_, FSep);
fl(2, :) = {FSep};
depth    = size(fl, 2);

% [..\Folder\]:
testFolder = ['Folder', FSep];
testStr    = testFolder;
for i = depth:-1:2
   testStr = ['..', FSep, testStr];  %#ok<AGROW>
   try
      cf  = GetFullPath(testStr);
      xfl = fl(:, 1:i - 1);
      
      % Construct the reply by hand:
      cf2 = cat(2, xfl{:}, testFolder);
      if strcmpi(cf, cf2)
         disp(['  ok: ', testStr]);
      else
         fprintf(['Path: [%s]  ==> error\n', ...
            '  GetFullPath replied: [%s]\n', ...
            '  Expected:            [%s]\n'], testStr, cf, cf2);
         error([FuncName, ': GetFullPath with folder failed']);
      end
   catch
      if isempty(lasterr)
         error(ErrID, [FuncName, ': GetFullPath crashed?!']);
      else
         error(ErrID, lasterr);
      end
   end
end

% [..\File\]:
testFile = 'File';
testStr  = testFile;
for i = depth:-1:2
   testStr = ['..', FSep, testStr];  %#ok<AGROW>
   try
      cf  = GetFullPath(testStr);
      xfl = fl(:, 1:i - 1);
      
      % Construct the reply by hand:
      cf2 = [cat(2, xfl{:}), testFile];
      if strcmpi(cf, cf2)
         disp(['  ok: ', testStr]);
      else
         fprintf(['Path: [%s]  ==> error\n', ...
            '  GetFullPath replied: [%s]\n', ...
            '  Expected:            [%s]\n'], testStr, cf, cf2);
         error(ErrID, [FuncName, ': GetFullPath with folder failed']);
      end
   catch
      if isempty(lasterr)
         error(ErrID, [FuncName, ': GetFullPath crashed?!']);
      else
         error(ErrID, lasterr);
      end
   end
end

% [PREFDIR\..\Folder\]:
testFolder = ['Folder', FSep];
testCore   = testFolder;
for i = depth:-1:2
   testCore = ['..', FSep, testCore];  %#ok<AGROW>
   testStr  = [pd_, FSep, testCore];
   try
      cf  = GetFullPath(testStr);
      xfl = fl(:, 1:i - 1);
      
      % Construct the reply by hand:
      cf2 = [cat(2, xfl{:}), testFolder];
      if strcmpi(cf, cf2)
         disp(['  ok: ', testStr]);
      else
         fprintf(['Path: [%s]  ==> error\n', ...
            '  GetFullPath replied: [%s]\n', ...
            '  Expected:            [%s]\n'], testStr, cf, cf2);
         error([FuncName, ': GetFullPath with folder failed']);
      end
   catch
      if isempty(lasterr)
         error(ErrID, [FuncName, ': GetFullPath crashed?!']);
      else
         error(ErrID, lasterr);
      end
   end
end

% [PREFDIR\..\File]:
testFile = 'File';
testCore = testFile;
for i = depth:-1:2
   testCore = ['..', FSep, testCore];  %#ok<AGROW>
   testStr  = [pd_, FSep, testCore];
   try
      cf  = GetFullPath(testStr);
      xfl = fl(:, 1:i - 1);
      
      % Construct the reply by hand:
      cf2 = [cat(2, xfl{:}), testFile];
      if strcmpi(cf, cf2)
         disp(['  ok: ', testStr]);
      else
         fprintf(['Path: [%s]  ==> error\n', ...
            '  GetFullPath replied: [%s]\n', ...
            '  Expected:            [%s]\n'], testStr, cf, cf2);
         error([FuncName, ': GetFullPath with folder failed']);
      end
   catch
      if isempty(lasterr)
         error(ErrID, [FuncName, ': GetFullPath crashed?!']);
      else
         error(ErrID, lasterr);
      end
   end
end

% [PREFDIR\..]:
testStr = pd_;
for i = depth:-1:1
   testStr = [testStr, FSep, '..'];  %#ok<AGROW>
   try
      cf = GetFullPath(testStr);
      
      % Construct the reply by hand:
      cf2 = CleanReply(testStr);
      if strcmpi(cf, cf2)
         disp(['  ok: ', testStr]);
      else
         fprintf(['Path: [%s]  ==> error\n', ...
            '  GetFullPath replied: [%s]\n', ...
            '  Expected:            [%s]\n'], testStr, cf, cf2);
         error([FuncName, ': GetFullPath with folder failed']);
      end
   catch
      if isempty(lasterr)
         error(ErrID, [FuncName, ': GetFullPath crashed?!']);
      else
         error(ErrID, lasterr);
      end
   end
end

% Check magic prefix under Windows: --------------------------------------------
if isWindows
   fprintf('\n== Test Magic prefix under Windows:\n');
   MAX_PATH = 260;
   
   % Test data folders do not need to exist:
   longName = repmat('a', 1, 250);
   testData = {'C:\Temp', fullfile('C:\Temp', longName, longName), ...
      '\\?\C:\Temp', fullfile('\\?\C:\Temp', longName, longName), ...
      '\\?\UNC\Server\Share', ...
      fullfile('\\?\UNC\Server\Share', longName, longName), ...
      '\\Server\Share', ...
      fullfile('\\Server\Share', longName, longName), ...
      'short', fullfile(longName, longName)};
   
   cd(tempdir);
   
   for iTestData = 1:numel(testData)
      folder   = testData{iTestData};  % Need not exist
      leanPath = GetFullPath(folder, 'lean');
      fatPath  = GetFullPath(folder, 'fat');
      autoPath = GetFullPath(folder, 'auto');
      
      fullWant = GetFullPath(folder);
      leanWant = strrep(strrep(fullWant, '\\?\UNC', '\'), '\\?\', '');
      uncIndex = strfind(fullWant, '\Server\Share');
      if any(uncIndex)
         fatWant = ['\\?\UNC', fullWant(uncIndex:length(fullWant))];
      else
         fatWant = ['\\?\', leanWant];
      end
      if length(leanWant) >= MAX_PATH
         autoWant = fatWant;
      else
         autoWant = leanWant;
      end
      
      if strcmp(leanPath, leanWant) == 0
         error(ErrID, '%s: Unexpected name for lean style?! [%s]', ...
            mfilename, leanPath);
      end
      if strcmp(autoPath, autoWant) == 0
         error(ErrID, '%s: Unexpected prefix for short name?! [%s]', ...
            mfilename, leanPath);
      end
      if strcmp(fatPath, fatWant) == 0
         error(ErrID, '%s: Unexpected prefix for fat style?! [%s]', ...
            mfilename, fatPath);
      end
      str = strrep(folder, longName, sprintf('<%d chars>', length(longName)));
      fprintf('  ok: %s\n', str);
      str = sprintf('      auto: %s\n      lean: %s\n      fat:  %s\n', ...
         autoPath, leanPath, fatPath);
      str = strrep(str, longName, sprintf('<%d chars>', length(longName)));
      fprintf('%s', str);
   end
else
   fprintf('\n== No magic prefix test under Linux/MacOS\n');
end

% Goodbye: ---------------------------------------------------------------------
cd(backDir);
fprintf('\n== %s passed the tests\n', examined);

% return;

% ******************************************************************************
function F = CleanReply(F_in)
% Remove \.. and \. from file name - slow, crude, stable!

FSep = filesep;

% Convert string to cell string:
C = Str2Cell_L(F_in, FSep);

% Leading slash for unix paths:
if isunix && isempty(C{1})
   C{1} = FSep;
end

C(strcmp(C, '.')) = [];
DDot              = find(strcmp(C, '..'));
while ~isempty(DDot)
   C(DDot(1)) = [];
   if DDot(1) > 2
      C(DDot(1) - 1) = [];
   end
   DDot = find(strcmp(C, '..'));
end

% Join cleaned cell string to a string again:
if length(C) > 1
   if F_in(length(F_in)) == FSep
      F = fullfile(C{:}, FSep);
   else
      F = fullfile(C{:});
   end
else
   F = fullfile(C{:}, FSep);
end

% return;

% ******************************************************************************
function CStr = Str2Cell_L(Str, Sep)
% Split string to cell string
% Use faster C-Mex Str2Cell for real problems.

persistent hasDataRead
if isempty(hasDataRead)
   hasDataRead = ~isempty('dataread');
end

if hasDataRead
   if strcmp(Sep, '\')
      Sep = '\\';
   end
   CStr = dataread('string', Str, '%s', 'delimiter', Sep, 'whitespace', '');
   CStr = reshape(CStr, 1, numel(CStr));
else
   CStr = regexp(Str, Sep, 'split');
end

% return;

% ******************************************************************************
function Cleanup(Arg)
persistent currentFolder
if nargin > 0
   currentFolder = Arg;
else
   cd(currentFolder);
end

% return;
