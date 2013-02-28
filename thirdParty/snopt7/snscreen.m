%function snscreen( filename )
%    Regulates output to the terminal.  Will print major iteration
%    information identical to that printed to a summary file if set.
%    Thus options that effect summary output may affect information
%    printed to the screen.  Option is off by default.  To turn on
%    the screen output, type:
%    >> snscreen on
%    to turn the screen output back off, type:
%    >> snscreen off
%
%    NOTE:  A summary file need not be set to use the screen option.
function snscreen( filename )

screenon     = 15;
screenoff    = 16;

if strcmp( filename, 'on' )
  snoptcmex( screenon );
elseif strcmp( filename, 'off' )
  snoptcmex( screenoff );
end
