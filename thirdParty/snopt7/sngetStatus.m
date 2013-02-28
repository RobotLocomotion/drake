% function [Status] = sngetStatus
%
% sngetStatus may be called from a user-defined function
% to determine the value of the snopt variable "Status".
%
% Possible values of Status are:
% Status =  0: There is nothing special about the current call.
% Status =  1: snopt is calling the user function for the first time.
% Status >= 2: snopt is calling the user function for the last time.
%
% For further details please see documentation for snoptA.
function [status] = sngetStatus( status )

getStatusOption = 19;
status = snoptcmex( getStatusOption );
