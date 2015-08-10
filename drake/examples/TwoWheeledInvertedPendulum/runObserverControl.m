function runObserverControl

r = TWIP();
[A,B] = r.linearize(0, zeros(4,1), 0);
A = full(A); B = full(B);
C = [0,0,1,0;1,-1,0,0]*180/pi;

%observer design by pole placement
omega0 = 25;
L = place(A', C', -omega0*[1,2,3,4])';
v = TWIPVisualizer(r);

%construct linear state observer
observer = LuenbergerObserver(r, L, []);
observer.forward_model = LinearSystem(A, B, [], [], C, []);

%construct LQR controller
controller = r.balanceLQR();

%wire up the system
sysCl = buildClosedLoopObserverControl(r, observer, controller);

xtraj = sysCl.simulate([0, 5]);
v.playback(xtraj)

function sysCl = buildClosedLoopObserverControl(plant, observer, controller)
    mdl_name = 'observer_control';
    new_system(mdl_name, 'Model');
    mdl = SimulinkModelHandle(mdl_name);
    mdl.addSubsystem('plant', getModel(plant));
    mdl.addSubsystem('controller', getModel(controller));
    mdl.addSubsystem('observer', getModel(observer));
    add_block('simulink3/Signals & Systems/Mux', [mdl_name, '/Mux']);
    mdl.add_line('controller/1', 'Mux/1');
    mdl.add_line('controller/1', 'plant/1');
    mdl.add_line('plant/1', 'Mux/2');
    mdl.add_line('Mux/1', 'observer/1');
    mdl.add_line('observer/1', 'controller/1');
    add_block('simulink3/Sinks/Out1',  [mdl_name, '/out']);
    mdl.add_line('observer/1', 'out/1');
    sysCl = SimulinkModel(mdl, 0);
    sysCl = sysCl.setOutputFrame(observer.getOutputFrame);
end

end