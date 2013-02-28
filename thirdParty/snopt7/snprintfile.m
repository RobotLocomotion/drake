% function snprintfile( filename )
%     Causes SNOPT to write detailed information about its progress
%     to the file named in "filename."
%
%     "snprintfile off" causes SNOPT to stop writing to filename,
%     and close the file.
%
%     Note that until the file has been closed, it may not contain
%     all of the output.
%
%     snprintfile serves the same function as snprint.
%
%     WARNING:  Do not use snset() or snseti() to set the print file.
function snprintfile( filename )

openprintfile  = 10;
closeprintfile = 12;

if strcmp( filename, 'off' )
  snoptcmex( closeprintfile );
elseif strcmp( filename, 'on' )
  snoptcmex( openprintfile, 'print.out' );
else
  snoptcmex( openprintfile, filename );
end
