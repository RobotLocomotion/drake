function megaclear() 
% Attempts to clear everything (classes, simulink, java, mex, ...) except breakpoints 

% Note: Must be a function so we can save breakpoint variables

    current_breakpoints = dbstatus('-completenames');

    force_close_system();
    
%    while ~isempty(vrgcf), close(vrgcf); end
    if usejava('awt'), vrclear; end
    
    evalin('base', 'h=findobj; delete(h(2:end));');  % delete dangling handles first
    evalin('base', 'clearvars -global');    % clear globals before
                                            % clearing mex (to help
                                            % with Hongkai's bug)
    evalin('base', 'clear all classes java imports mex');    % now clear everything else

    dbstop(current_breakpoints);
    
end
