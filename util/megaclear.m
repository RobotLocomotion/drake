function megaclear() 
% Attempts to clear everything (classes, simulink, java, mex, ...) except breakpoints 

% Note: Must be a function so we can save breakpoint variables

    current_breakpoints = dbstatus('-completenames');

    force_close_system();
    vrclose all;
    vrclear;
    
    evalin('base', 'h=findobj; delete(h(2:end));');  % delete dangling handles first
    evalin('base', 'clear all classes java imports mex');    % now clear everything else

    dbstop(current_breakpoints);
    
end
