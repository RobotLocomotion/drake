%
% snopt m-files
% -------------
%
% snopt.m
%     is a simple mex interface to the Fortran routine snOptA.  This
%     routine finds the minimum of a smooth nonlinear function subject to
%     linear and nonlinear constraints.
%
%     snopt.m may be called in one of four ways, depending on the number
%     of arguments in the calling sequence.  In general, the robustness
%     increases with the number of arguments (and complexity).
%
%     The user need not provide the derivatives of the problem functions,
%     although user-provided derivatives increase the speed and
%     reliability of snopt.  The problem derivatives may be provided
%     in either dense or sparse format.
%
%     For examples of the three simplest ways to invoke snopt.m, see the
%     files:
%         examples/snmain/snoptmain.m   examples/snmain/snoptuserfun.m
%         examples/snmain/snoptmain2.m  examples/snmain/snoptuserfun2.m
%         examples/snmain/snoptmain3.m  examples/snmain/snoptuserfun3.m
%         examples/snmain/snoptmain2.m  examples/snmain/snoptuserfun2.m
%
% snsolve.m
%     calls the same optimization routine as snopt, but has more flexibility
%     in specifying derivatives.  The arguments are similar to those
%     of the Fortran routine snOptA.
%
%     For examples of invocations of snsolve.m, see the files:
%         examples/snmain/snsolvemain.m  examples/snmain/snsolveuserfun.m
%         examples/sntoy/sntoy.m         examples/sntoy/toyusrfun.m
%         examples/sntoy/sntoy2.m        examples/sntoy/toyusrfun2.m
%         examples/hsmain/hsmain.m       examples/hsmain/hsmainusrfun2.m
%
%
% snscreen.m
%     starts and stops a copy of the summary output being printed to the
%     Matlab terminal display.
%
%     To start output to the screen, type:
%       >> snscreen on;
%     To stop  output to the screen, type:
%       >> snscreen off;
%     Screen output may be simultaneously appended to a file (see "snsummary").
%
%
% snsummary.m
%     saves summary information to a file.
%       >> snsummary ('probName.sum');   % Starts printing on probName.sum
%       >> snsummary   off;              % Closes the summary file
%
%     Note: data is appended to the summary file.
%     It may be necessary to close the summary file to purge the summary buffer.
%
%
% snprint.m (identical to snprintfile.m)
%     saves detailed information about the progress of snopt to a file.
%       >> snprint   ('probName.out');   % Starts printing on probName.out
%       >> snprint    off;               % Closes the print file
%
%     It may be necessary to close the print file to purge the print buffer.
%
%
% snspec.m
%     reads an options file for the snopt optimization routine.
%       >> t1diet.spc = which('t1diet.spc');
%       >> snspec ( t1diet.spc );
%
%
% snset, snseti, snsetr
%     set advanced options for the snopt optimization routine.
%
%
% snget, sngeti, sngetr
%     retrieve current values of snopt optional parameters.
%     (they have the opposite effect of snset, snseti, and snsetr)
%
%
% sngetStatus.m
%     can be invoked inside the user-defined function to determine if
%     the problem functions are being called for the first time.
%
%
% snsetStatus.m
%     can be invoked inside of the user-defined function
%     to signal that the objective function or constraints are undefined
%     at the current value of x.  snopt will choose another point closer
%     to the best point found so far.
%
%
% snJac
%     finds the coordinate structure for sparse derivatives.
%
%
% Philip Gill and Joshua Griffin,  UCSD  19 October 2004
% Updated                                 4 May     2006
%                                        18 Jun     2007
