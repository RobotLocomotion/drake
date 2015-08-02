function File = GetFullPath(File, Style)
% GetFullPath - Get absolute canonical path of a file or folder
% Absolute path names are safer than relative paths, when e.g. a GUI or TIMER
% callback changes the current directory. Only canonical paths without "." and
% ".." can be recognized uniquely.
% Long path names (>259 characters) require a magic initial key "\\?\" to be
% handled by Windows API functions, e.g. for Matlab's FOPEN, DIR and EXIST.
%
% FullName = GetFullPath(Name, Style)
% INPUT:
%   Name:  String or cell string, absolute or relative name of a file or
%          folder. The path need not exist. Unicode strings, UNC paths and long
%          names are supported.
%   Style: Style of the output as string, optional, default: 'auto'.
%          'auto': Add '\\?\' or '\\?\UNC\' for long names on demand.
%          'lean': Magic string is not added.
%          'fat':  Magic string is added for short names also.
%          The Style is ignored when not running under Windows.
%
% OUTPUT:
%   FullName: Absolute canonical path name as string or cell string.
%          For empty strings the current directory is replied.
%          '\\?\' or '\\?\UNC' is added on demand.
%
% NOTE: The M- and the MEX-version create the same results, the faster MEX
%   function works under Windows only.
%   Some functions of the Windows-API still do not support long file names.
%   E.g. the Recycler and the Windows Explorer fail even with the magic '\\?\'
%   prefix. Some functions of Matlab accept 260 characters (value of MAX_PATH),
%   some at 259 already. Don't blame me.
%   The 'fat' style is useful e.g. when Matlab's DIR command is called for a
%   folder with les than 260 characters, but together with the file name this
%   limit is exceeded. Then "dir(GetFullPath([folder, '\*.*], 'fat'))" helps.
%
% EXAMPLES:
%   cd(tempdir);                    % Assumed as 'C:\Temp' here
%   GetFullPath('File.Ext')         % 'C:\Temp\File.Ext'
%   GetFullPath('..\File.Ext')      % 'C:\File.Ext'
%   GetFullPath('..\..\File.Ext')   % 'C:\File.Ext'
%   GetFullPath('.\File.Ext')       % 'C:\Temp\File.Ext'
%   GetFullPath('*.txt')            % 'C:\Temp\*.txt'
%   GetFullPath('..')               % 'C:\'
%   GetFullPath('..\..\..')         % 'C:\'
%   GetFullPath('Folder\')          % 'C:\Temp\Folder\'
%   GetFullPath('D:\A\..\B')        % 'D:\B'
%   GetFullPath('\\Server\Folder\Sub\..\File.ext')
%                                   % '\\Server\Folder\File.ext'
%   GetFullPath({'..', 'new'})      % {'C:\', 'C:\Temp\new'}
%   GetFullPath('.', 'fat')         % '\\?\C:\Temp\File.Ext'
%
% COMPILE:
%   Automatic: InstallMex GetFullPath.c uTest_GetFullPath
%   Manual:    mex -O GetFullPath.c
%   Download:  http://www.n-simon.de/mex
% Run the unit-test uTest_GetFullPath after compiling.
%
% Tested: Matlab 6.5, 7.7, 7.8, 7.13, WinXP/32, Win7/64
%         Compiler: LCC2.4/3.8, BCC5.5, OWC1.8, MSVC2008/2010
% Assumed Compatibility: higher Matlab versions
% Author: Jan Simon, Heidelberg, (C) 2009-2013 matlab.THISYEAR(a)nMINUSsimon.de
%
% See also: CD, FULLFILE, FILEPARTS.

% $JRev: R-G V:032 Sum:7Xd/JS0+yfax Date:15-Jan-2013 01:06:12 $
% $License: BSD (use/copy/change/redistribute on own risk, mention the author) $
% $UnitTest: uTest_GetFullPath $
% $File: Tools\GLFile\GetFullPath.m $
% History:
% 001: 20-Apr-2010 22:28, Successor of Rel2AbsPath.
% 010: 27-Jul-2008 21:59, Consider leading separator in M-version also.
% 011: 24-Jan-2011 12:11, Cell strings, '~File' under linux.
%      Check of input types in the M-version.
% 015: 31-Mar-2011 10:48, BUGFIX: Accept [] as input as in the Mex version.
%      Thanks to Jiro Doke, who found this bug by running the test function for
%      the M-version.
% 020: 18-Oct-2011 00:57, BUGFIX: Linux version created bad results.
%      Thanks to Daniel.
% 024: 10-Dec-2011 14:00, Care for long names under Windows in M-version.
%      Improved the unittest function for Linux. Thanks to Paul Sexton.
% 025: 09-Aug-2012 14:00, In MEX: Paths starting with "\\" can be non-UNC.
%      The former version treated "\\?\C:\<longpath>\file" as UNC path and
%      replied "\\?\UNC\?\C:\<longpath>\file".
% 032: 12-Jan-2013 21:16, 'auto', 'lean' and 'fat' style.

% Initialize: ==================================================================
% Do the work: =================================================================

% #############################################
% ### USE THE MUCH FASTER MEX ON WINDOWS!!! ###
% #############################################

% Difference between M- and Mex-version:
% - Mex does not work under MacOS/Unix.
% - Mex calls Windows API function GetFullPath.
% - Mex is much faster.

% Magix prefix for long Windows names:
if nargin < 2
   Style = 'auto';
end

% Handle cell strings:
% NOTE: It is faster to create a function @cell\GetFullPath.m under Linux, but
% under Windows this would shadow the fast C-Mex.
if isa(File, 'cell')
   for iC = 1:numel(File)
      File{iC} = GetFullPath(File{iC}, Style);
   end
   return;
end

% Check this once only:
isWIN    = strncmpi(computer, 'PC', 2);
MAX_PATH = 260;

% Warn once per session (disable this under Linux/MacOS):
persistent hasDataRead
if isempty(hasDataRead)
   % Test this once only - there is no relation to the existence of DATAREAD!
   %if isWIN
   %   Show a warning, if the slower Matlab version is used - commented, because
   %   this is not a problem and it might be even useful when the MEX-folder is
   %   not inlcuded in the path yet.
   %   warning('JSimon:GetFullPath:NoMex', ...
   %      ['GetFullPath: Using slow Matlab-version instead of fast Mex.', ...
   %       char(10), 'Compile: InstallMex GetFullPath.c']);
   %end
   
   % DATAREAD is deprecated in 2011b, but still available. In Matlab 6.5, REGEXP
   % does not know the 'split' command, therefore DATAREAD is preferred:
   hasDataRead = ~isempty(which('dataread'));
end

if isempty(File)  % Accept empty matrix as input:
   if ischar(File) || isnumeric(File)
      File = cd;
      return;
   else
      error(['JSimon:', mfilename, ':BadTypeInput1'], ...
         ['*** ', mfilename, ': Input must be a string or cell string']);
   end
end

if ischar(File) == 0  % Non-empty inputs must be strings
   error(['JSimon:', mfilename, ':BadTypeInput1'], ...
      ['*** ', mfilename, ': Input must be a string or cell string']);
end

if isWIN  % Windows: --------------------------------------------------------
   FSep = '\';
   File = strrep(File, '/', FSep);
   
   % Remove the magic key on demand, it is appended finally again:
   if strncmp(File, '\\?\', 4)
      if strncmpi(File, '\\?\UNC\', 8)
         File = ['\', File(7:length(File))];  % Two leading backslashes!
      else
         File = File(5:length(File));
      end
   end
   
   isUNC   = strncmp(File, '\\', 2);
   FileLen = length(File);
   if isUNC == 0                        % File is not a UNC path
      % Leading file separator means relative to current drive or base folder:
      ThePath = cd;
      if File(1) == FSep
         if strncmp(ThePath, '\\', 2)   % Current directory is a UNC path
            sepInd  = strfind(ThePath, '\');
            ThePath = ThePath(1:sepInd(4));
         else
            ThePath = ThePath(1:3);     % Drive letter only
         end
      end
      
      if FileLen < 2 || File(2) ~= ':'  % Does not start with drive letter
         if ThePath(length(ThePath)) ~= FSep
            if File(1) ~= FSep
               File = [ThePath, FSep, File];
            else                        % File starts with separator:
               File = [ThePath, File];
            end
         else                           % Current path ends with separator:
            if File(1) ~= FSep
               File = [ThePath, File];
            else                        % File starts with separator:
               ThePath(length(ThePath)) = [];
               File = [ThePath, File];
            end
         end
         
      elseif FileLen == 2 && File(2) == ':'   % "C:" current directory on C!
         % "C:" is the current directory on the C-disk, even if the current
         % directory is on another disk! This was ignored in Matlab 6.5, but
         % modern versions considers this strange behaviour.
         if strncmpi(ThePath, File, 2)
            File = ThePath;
         else
            try
               File = cd(cd(File));
            catch    % No MException to support Matlab6.5...
               if exist(File, 'dir')  % No idea what could cause an error then!
                  rethrow(lasterror);
               else  % Reply "K:\" for not existing disk:
                  File = [File, FSep];
               end
            end
         end
      end
   end
   
else         % Linux, MacOS: ---------------------------------------------------
   FSep = '/';
   File = strrep(File, '\', FSep);
   
   if strcmp(File, '~') || strncmp(File, '~/', 2)  % Home directory:
      HomeDir = getenv('HOME');
      if ~isempty(HomeDir)
         File(1) = [];
         File    = [HomeDir, File];
      end
      
   elseif strncmpi(File, FSep, 1) == 0
      % Append relative path to current folder:
      ThePath = cd;
      if ThePath(length(ThePath)) == FSep
         File = [ThePath, File];
      else
         File = [ThePath, FSep, File];
      end
   end
end

% Care for "\." and "\.." - no efficient algorithm, but the fast Mex is
% recommended at all!
if ~isempty(strfind(File, [FSep, '.']))
   if isWIN
      if strncmp(File, '\\', 2)  % UNC path
         index = strfind(File, '\');
         if length(index) < 4    % UNC path without separator after the folder:
            return;
         end
         Drive            = File(1:index(4));
         File(1:index(4)) = [];
      else
         Drive     = File(1:3);
         File(1:3) = [];
      end
   else  % Unix, MacOS:
      isUNC   = false;
      Drive   = FSep;
      File(1) = [];
   end
   
   hasTrailFSep = (File(length(File)) == FSep);
   if hasTrailFSep
      File(length(File)) = [];
   end
   
   if hasDataRead
      if isWIN  % Need "\\" as separator:
         C = dataread('string', File, '%s', 'delimiter', '\\');  %#ok<REMFF1>
      else
         C = dataread('string', File, '%s', 'delimiter', FSep);  %#ok<REMFF1>
      end
   else  % Use the slower REGEXP, when DATAREAD is not available anymore:
      C = regexp(File, FSep, 'split');
   end
   
   % Remove '\.\' directly without side effects:
   C(strcmp(C, '.')) = [];
   
   % Remove '\..' with the parent recursively:
   R = 1:length(C);
   for dd = reshape(find(strcmp(C, '..')), 1, [])
      index    = find(R == dd);
      R(index) = [];
      if index > 1
         R(index - 1) = [];
      end
   end
   
   if isempty(R)
      File = Drive;
      if isUNC && ~hasTrailFSep
         File(length(File)) = [];
      end
      
   elseif isWIN
      % If you have CStr2String, use the faster:
      %   File = CStr2String(C(R), FSep, hasTrailFSep);
      File = sprintf('%s\\', C{R});
      if hasTrailFSep
         File = [Drive, File];
      else
         File = [Drive, File(1:length(File) - 1)];
      end
      
   else  % Unix:
      File = [Drive, sprintf('%s/', C{R})];
      if ~hasTrailFSep
         File(length(File)) = [];
      end
   end
end

% "Very" long names under Windows:
if isWIN
   if ~ischar(Style)
      error(['JSimon:', mfilename, ':BadTypeInput2'], ...
         ['*** ', mfilename, ': Input must be a string or cell string']);
   end
   
   if (strncmpi(Style, 'a', 1) && length(File) >= MAX_PATH) || ...
         strncmpi(Style, 'f', 1)
      % Do not use [isUNC] here, because this concerns the input, which can
      % '.\File', while the current directory is an UNC path.
      if strncmp(File, '\\', 2)  % UNC path
         File = ['\\?\UNC', File(2:end)];
      else
         File = ['\\?\', File];
      end
   end
end

% return;
