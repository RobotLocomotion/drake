% function  set = snget( option )
%     The  optional INTEGER-valued parameter defined by the
%     string "option" is assigned to rvalue.
%
%     For a description of the optional parameters, see
%     the snopt documentation.
%
function set = snget( option )

getoption = 5;
set = snoptcmex( getoption, option );
