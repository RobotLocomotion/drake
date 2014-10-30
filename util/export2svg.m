function export2svg(filename,varargin)
% exports current figure to filename.svg
% @param filename should have no extension... pdf will be added
% additional arguments are the same as for export2pdf

% note: tried plot2svg from matlabcentral, but it didn't work nearly as well

tmp = tempname;
export2pdf(tmp,varargin{:});

if (systemWCMakeEnv(['pdf2svg ', tmp, '.pdf ', filename, '.svg']) ~= 0) 
  error('pdf2svg failed.'); 
end

delete([tmp,'.pdf']);

