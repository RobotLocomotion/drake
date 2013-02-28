% function snsetStatus( status )
%
% snsetStatus may be called from the user-defined function
% to signal snopt that the current value of x is infeasible.
% Possible values of status:
% status < -1: Signals snopt to stop.
% status = -1: Functions are undefined at x, the line search will
%              shorten the step and try again.
% status =  0: There is nothing special about current x.
%
% For further details see the documentation for snoptA.
function snsetStatus( status )

setStatusOption = 18;
snoptcmex( setStatusOption, status );
