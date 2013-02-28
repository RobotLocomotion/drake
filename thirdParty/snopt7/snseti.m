% function snseti( option, ivalue )
%     Sets the INTEGER-VALUED optional parameter defined by the
%     string "option" is assigned to rvalue.
%
%     For a description of all the optional parameters, see the
%     snopt documentation.
%
%     Do not try to set the unit number of the summary or print file.
%     Use the MATLAB functions snsummary and snprintfile instead.
%
function snseti( option, ivalue )

setoptionI = 3;
snoptcmex( setoptionI, option, ivalue );
