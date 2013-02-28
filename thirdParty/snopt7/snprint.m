% function snprint( filename )
%     Causes SNOPT to write detailed information about its progress
%     to the file named in "filename."
%
%     "snprint off" causes SNOPT to stop writing to filename,
%     and close the file.
%
%     Note that until the file has been closed, it may not contain
%     all of the output.
%
%     snprint serves the same function as snprintfile.
%
%     WARNING:  Do not use snset() or snseti() to set the print file.

function snprint( filename )

%openprintfile  = snoptcmex( 0, 'SetPrintFile'   );
%closeprintfile = snoptcmex( 0, 'ClosePrintFile' );

openprintfile  = 10;
closeprintfile = 12;

if strcmp( filename, 'off' )
  snoptcmex( closeprintfile );
elseif strcmp( filename, 'on' )
  snoptcmex( openprintfile, 'print.out' );
else
  snoptcmex( openprintfile, filename );
end
