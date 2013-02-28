% function snset( option )
%     Sets a optional parameter of snopt. The string "option" will be read
%     by snopt. If the string contains a setting that snopt understands,
%     snopt will set internal parameters accordingly. For a description of
%     available parameters, please see the snopt documentation.
%
%     Do not try to set the unit number of the summary or print file.
%     Use the MATLAB functions snsummary and snprintfile instead.
%
function snset( option )

setoption = 2;
snoptcmex( setoption, option );