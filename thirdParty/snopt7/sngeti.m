% function  ivalue = sngeti( option )
%     The  optional INTEGER-valued parameter defined by the
%     string "option" is assigned to rvalue.
%
%     For a description of the optional parameters, see
%     the snopt documentation.
%
function ivalue = sngeti( option )

getoptionI = 7;
ivalue = snoptcmex( getoptionI, option );
