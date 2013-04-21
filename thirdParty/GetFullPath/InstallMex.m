function InstallMex(SourceFile, varargin)
% Compile a C-Mex file
% This function calls MEX() to compile a C/C++/F-Mex source file.
% Advanced users can call "mex -O SourceFile.c" instead, but beginners are
% sometimes overwhelmed by compiling instructions.
%
% InstallMex(SourceFile, ...)
% INPUT:
%   SourceFile:  Name of the source file with extension, with or without
%                absolute or partial path.
%   Optional arguments:
%   - If the name of a unit-test function is provided as string, it is called
%     after the compilation.
%   - Additional arguments for the compilation can be defined as cell string.
%   - If the compilation fails, add the string '-debug' to get more information.
%
% OUTPUT: none.
%
% COMPATIBILITY:
% - You have to install a compiler at first and enable it by: mex -setup
% - For Linux and MacOS the C99 style is enabled.
% - This function has been tested under Windows only.
%
% EXAMPLE:
%   Compile fcn.c with LAPACK libraries:
%     InstallMex('fcn', {'libmwlapack.lib', 'libmwblas.lib'})
%
% Suggestions for improvements and comments are welcome!
% Feel free to add this function to other FEX submissions, when you change the
% "Precompiled" URL accordingly.
%
% Tested: Matlab 6.5, 7.7, 7.8, 7.13, WinXP/32, Win7/64
% Author: Jan Simon, Heidelberg, (C) 2012-2013 matlab.THISYEAR(a)nMINUSsimon.de

% $JRev: R-m V:012 Sum:B0OqUrOSygTE Date:17-Jan-2013 00:43:49 $
% $License: BSD (use/copy/change/redistribute on own risk, mention the author) $
% $File: Tools\GLMisc\InstallMex.m $
% History:
% 001: 27-Jul-2012 09:06, First version.
% 005: 29-Jul-2012 17:11, Run the unit-test instead of showing a link only.
% 006: 11-Aug-2012 23:59, Flexible treatment of inputs.

% Initialize: ==================================================================
% Global Interface: ------------------------------------------------------------
% URL to pre-compiled files - set to empty string, if there is none:
Precompiled = 'http://www.n-simon.de/mex';

% Initial values: --------------------------------------------------------------
bakCD = cd;

% Program Interface: -----------------------------------------------------------
% Parse inputs:
Param       = {};
UnitTestFcn = '';
doDebug     = false;

% First input is the name of the source file. If it is omitted or empty, open
% a dialog to choose a file:
if nargin == 0 || isempty(SourceFile)
   cd(fileparts(mfilename('fullpath')));
   [SourceName, SourcePath] = uigetfile( ...
      {'*.c;*.cpp;*.f', 'Source files (.c, .cpp, .f)'; ...
      '*.*',            'All Files (*.*)'}, ...
      'Pick a file to compile');
   cd(bakCD);
   if ~ischar(SourceName)  % User cancelled the dialog:
      return;
   end
   SourceFile = fullfile(SourcePath, SourceName);

elseif ~ischar(SourceFile)
   error(['JSimon:', mfilename, ':BadTypeInput1'], ...
      '*** %s: 1st input must be a string.', mfilename);
end

% Additional inputs are identified bytheir type:
% String:      unit-test function or the flag to enable debugging.
% Cell string: additional parameters for the MEX command
for iArg = 1:numel(varargin)
   Arg = varargin{iArg};
   if ischar(Arg)
      if strcmpi(Arg, '-debug')
         doDebug = true;
      elseif exist(Arg, 'file') == 2
         UnitTestFcn = Arg;
      else
         error(['JSimon:', mfilename, ':MissFile'], ...
            '*** %s: Unknown string or missing file: %s', mfilename, Arg);
      end
   elseif iscellstr(Arg)
      Param = Arg;
   else
      error(['JSimon:', mfilename, ':BadInputType'], ...
         '*** %s: Bad type of input.', mfilename);
   end
end

% User Interface: --------------------------------------------------------------
% Display hyper-links in the command window:
hasHRef = usejava('jvm');

% Do the work: =================================================================
% Search the C-sources, solve partial or relative paths:
whichSource = which(SourceFile);
if isempty(whichSource)
   error(['JSimon:', mfilename, ':NoSource'], ...
      '*** %s: Cannot find the C-file: %s', mfilename, SourceFile);
end

[SourcePath, SourceName, Ext] = fileparts(whichSource);
Source = [SourceName, Ext];

fprintf('== Compile: %s\n', fullfile(SourcePath, Source));

% Check if the compiled file is existing already:
whichMex = which([SourceName, '.', mexext]);
if ~isempty(whichMex)
   fprintf('::: Compiled file is existing already:\n    %s\n', whichMex);
   return;
end

if isunix && strcmpi(Ext, '.c')
   % Enable the "modern" C99 style. Matlab defines the archaic C89 style as
   % default for the GCC compiler. If you think, that this is hilarious, send an
   % enhancement report to support@mathworks.com.
   % Note: 'CFLAGS="\$CFLAGS -std=c99"' must be separated to 2 strings!!!
   Flags = {'-O', 'CFLAGS="\$CFLAGS', '-std=c99"', Source};
else
   Flags = {'-O', Source};
end

% Large array dimensions under 64 bit:
if [100, 1] * sscanf(version, '%d.%d', 2) >= 705
   if any(strfind(computer, '64'))
      Flags = cat(2, Flags, {'-largeArrayDims'});
   else
      Flags = cat(2, Flags, {'-compatibleArrayDims'});
   end
end

% Debug mode:
if doDebug && ~any(strcmp(Param, '-v'))
   Flags = cat(2, {'-v'}, Flags);
end

% Compile: ---------------------------------------------------------------------
% Display the compilation command:
Flags = cat(1, Flags(:), Param(:));
cmd   = ['mex', sprintf(' %s', Flags{:})];
fprintf('  %s\n', cmd);
   
try
   % Start the compilation:
   cd(SourcePath);
   mex(Flags{:});
   fprintf('Success:\n  %s\n', which([SourceName, '.', mexext]));
   compiled = true;
   
catch  % No MException to support Matlab 6.5
   compiled = false;
   err      = lasterror;
   fprintf(2, '\n*** Compilation failed:\n%s\n', err.message);
   if hasHRef
      fprintf(['Setup the compiler on demand:\n', ...
         '<a href="matlab:mex -setup">mex -setup</a>\n']);
      fprintf(['And/or try to compile manually:\n', ...
         '  cd(''%s'')\n  %s -v\n'], SourcePath, cmd);
      if ~isempty(Precompiled)
         fprintf('Or download the pre-compiled mex:\n');
         fprintf('  <a href="web:%s">%s</a>\n', Precompiled, Precompiled);
      end
   else
      fprintf('Setup the compiler on demand:\n  mex -setup\n');
      fprintf(['And/or try to compile manually:\n', ...
         '  cd(''%s'')\n  %s -v\n'], SourcePath, cmd);
      if ~isempty(Precompiled)
         fprintf('Or download the pre-compiled mex:\n  %s\n', Precompiled);
      end
   end
end

% Restore original directory:
cd(bakCD);

% Run the unit-test: -----------------------------------------------------------
if ~isempty(UnitTestFcn) && compiled
   [dum, UnitTestName] = fileparts(UnitTestFcn);  %#ok<ASGLU> % Remove extension
   if ~isempty(which(UnitTestName))
      str = repmat('=', 1, 10);
      fprintf('\n\n%s Run unit-test: %s %s\n', str, UnitTestName, str);
      feval(UnitTestName);
   else
      fprintf(2, '??? Cannot find unit-test: %s\n', UnitTestFcn);
   end
end

% return;
