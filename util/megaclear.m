function megaclear()
% Attempts to clear everything (classes, simulink, java, mex, ...) except breakpoints

% Note: Must be a function so we can save breakpoint variables

    current_breakpoints = dbstatus('-completenames');

    force_close_system();

%    while ~isempty(vrgcf), close(vrgcf); end
    if usejava('awt'), vrclear; end

    evalin('base', 'h=findobj; if numel(h)>1, delete(h(2:end)); end');  % delete dangling handles first
    evalin('base', 'clear all classes java imports');    % now clear everything else
    evalin('base', 'clear mex');  

    dbstop(current_breakpoints);

end
