% function inform = snspec( filename )
%     Causes snopt to read its optional parameters from the named file.
%     The format of this file is described in the snopt documentation.
%
%     Returns 0 if successful, and a positive number otherwise.
function inform = snspec( filename )

snoption = 9;
inform   = snoptcmex( snoption, filename );
