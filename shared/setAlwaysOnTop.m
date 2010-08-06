function setAlwaysOnTop(hFig,bool)
%SETALWAYSONTOP  Changes the always-on-top window state.
%   SETALWAYSONTOP(HFIG,TRUE) will make Matlab figure with handle HFIG to 
%   be on top of other windows in the OS even though it might not be in 
%   focus.
%   SETALWAYSONTOP(HFIG,FALSE) will put figure back to normal window state.
%   SETALWAYSONTOP(HFIG) is the same as SETALWAYSONTOP(HFIG,TRUE).
%   Second boolean argument TRUE/FALSE can also be exchanged to numerical
%   1/0.
%   
%   Restrictions:
%       HFIG must have property value: Visible = on.
%       HFIG must have property value: WindowStyle = normal.
%       Swing components must be available in the current Matlab session.
%   
%   Example:
%       hFig = figure;
%       setAlwaysOnTop(hFig,true); % figure is now on top of other windows
%
%   See also FIGURE.

%   Developed by Per-Anders Ekström, 2003-2007 Facilia AB.

error(nargchk(1,2,nargin))
error(javachk('swing',mfilename)) % Swing components must be available.
if nargin==1
    bool = true;
end
if ~ishandle(hFig) || length(hFig)~=1 || ~strcmp('figure',get(hFig,'Type'))
    error('SETALWAYSONTOP:fighandle','First Arg. Must be a Figure-Handle')
end
if strcmp('off',get(hFig,'Visible'))
    error('SETALWAYSONTOP:figvisible','Figure Must be Visible')
end
if ~strcmp('normal',get(hFig,'WindowStyle'))
    error('SETALWAYSONTOP:figwindowstyle','WindowStyle Must be Normal')
end
if isnumeric(bool)
    warning off MATLAB:conversionToLogical
    bool = logical(bool);
    warning on MATLAB:conversionToLogical
end
if ~islogical(bool) || length(bool)~=1
    error('SETALWAYSONTOP:boolean','Second Arg. Must be a Boolean Scalar')
end

% Flush the Event Queue of Graphic Objects and Update the Figure Window.
drawnow expose

% Get JavaFrame of Figure.
fJFrame = get(hFig,'JavaFrame');

% Set JavaFrame Always-On-Top-Setting.
fJFrame.fFigureClient.getWindow.setAlwaysOnTop(bool);
