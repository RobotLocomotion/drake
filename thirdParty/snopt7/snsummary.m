%function snsummary( filename )
%     Causes SNOPT to write summarized information about its progress
%     to the file named in "filename".
%
%     "snsummary off" causes SNOPT to stop writing to filename,
%     and close the file.
%
%     Note that until the file has been closed, it may not contain
%     all of the output.
%
%     WARNING:  Do not use snset() or snseti() to set the summary file.
function snsummary( filename )

opensummary  = 11;
closesummary = 13;

if strcmp( filename, 'off' )
  snoptcmex( closesummary );
else
  snoptcmex( opensummary, filename );
end
