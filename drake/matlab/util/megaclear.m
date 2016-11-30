function megaclear()
% Attempts to clear everything (classes, simulink, java, mex, ...) except breakpoints

% Note: Must be a function so we can save breakpoint variables

    current_breakpoints = dbstatus('-completenames');

    force_close_system();

    % Even if no VR worlds have ever been constructed, calling vrwho() and/or
    % vrclear makes subsequent calls to evalin('base', 'clear all classes java
    % imports'); incredibly slow (on the order of 20 seconds). We get around
    % that by checking an environment variable which is set in
    % RigidBodyWRLVisualizer.
    if getenv('HAS_MATLAB_VRWORLD')
        if usejava('awt')
          vrclose;
          vrclear; 
        end
        setenv('HAS_MATLAB_VRWORLD', '');
    end

    evalin('base', 'h=findobj; if numel(h)>1, delete(h(2:end)); end');  % delete dangling handles first
    evalin('base', 'clear all classes java imports');    % now clear everything else
    evalin('base', 'clear mex');  
    

    dbstop(current_breakpoints);

end
