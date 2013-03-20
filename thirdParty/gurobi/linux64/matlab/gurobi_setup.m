function gurobi_setup()
% gurobi_setup Setup and test the Gurobi Matlab interface

% Numeric version
ver = version;
ver(ver=='.') = ' ';
ver = sscanf(ver,'%d');

if (ver(1) ~= 7 && ver(1) ~= 8) || ...
   (ver(1) == 7 && ver(2) <  7) || ...
   (ver(1) == 8 && ver(2) > 0)
    warning('gurobi_setup:MATLABver', ...
            ['You are using MATLAB %s.\n' ...
             'The Gurobi MATLAB interface currently supports ' ...
             'R2008b-R2012b.'], version);
end

if ispc
   fs = '\';
else
   fs = '/';
end

gurobipath = mfilename('fullpath');
slashes = strfind(gurobipath, fs);
gurobipath = gurobipath(1:slashes(end));

% Check for GUROBI_HOME on Windows and Linux to ensure Gurobi has been
% correctly installed
if ~ismac
    gurobihome = getenv('GUROBI_HOME');
    if isempty(gurobihome)
        error('gurobi_setup:BadEnv', ...
            ['The Gurobi environmental variables are not set.\n' ...
            'Please check that Gurobi is installed correctly, '...
            'and these variables are visible within MATLAB.'])
    end
    gurobihome = strtrim(gurobihome);
    % remove trailing slash
    if gurobihome(end) == fs
        gurobihome(end) = [];
    end
    slashes = strfind(gurobihome, fs);
    sep = slashes(end)+1;
    gurobiarch = gurobihome(sep:end);

else
    % We don't require GUROBI_HOME on the mac. Get gurobiarch from path.
    gurobiarch = gurobipath;
    gurobiarch(end) = [];
    slashes = strfind(gurobiarch, fs);
    gurobiarch = gurobiarch(slashes(end-1)+1:slashes(end)-1);
end


matlabarch = computer;


switch matlabarch
  case 'PCWIN'
    if ~strcmp(gurobiarch, 'win32')
        error(['GUROBI architecture %s does not match matlab ' ...
               'architecture %s'], gurobiarch, matlabarch);
    end
  case 'PCWIN64'
    if ~strcmp(gurobiarch, 'win64')
        error(['GUROBI architecture %s does not match matlab ' ...
               'architecture %s'], gurobiarch, matlabarch);
    end
  case 'GLNXA64'
    if ~strcmp(gurobiarch, 'linux64')
        error(['GUROBI architecture %s does not match matlab ' ...
               'architecture %s'], gurobiarch, matlabarch);
    end
  case 'MACI64'
    if ~strcmp(gurobiarch, 'mac64')
        error(['GUROBI architecture %s does not match matlab ' ...
               'architecture %s'], gurobiarch, matlabarch);
    end
  otherwise
    error('Unsupported MATLAB architecture %s', matlabarch);
end

addpath(gurobipath)

try
    gurobi(1)
catch me
    if ~strcmp(me.identifier, 'gurobi:BadInput')
        throw(me)
    end
end

% gurobi.mex* is loaded, now test for license issues
model.obj = 1;
model.A  = sparse(1,1);
model.rhs = 0;
model.sense = '=';
params.outputflag = false;
result = gurobi(model, params);

fprintf('\nThe MATLAB interface for Gurobi %d.%d.%d has been installed.\n\n', ...
        result.versioninfo.major, result.versioninfo.minor, ...
        result.versioninfo.technical);


disp('The directory')
disp(['    ' gurobipath])
disp('has been added to the MATLAB path.')
disp('To use Gurobi regularly, you must save this new path definition.');
disp('To do this, type the command' );
disp('    savepath' );
disp('at the MATLAB prompt. Please consult the MATLAB documentation');
disp('if necessary.');
