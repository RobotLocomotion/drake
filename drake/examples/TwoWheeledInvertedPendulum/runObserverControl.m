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

ytraj = sysCl.simulate([0, 5]);
v.playback(ytraj)

function sysCl = buildClosedLoopObserverControl(plant, observer, controller)
  output_select(1).system = 1;
  output_select(1).output = 1;
  output_select(2).system = 2;
  output_select(2).output = 1;
  sys = mimoCascade(controller,plant,[],[],output_select);

  clear output_select;
  output_select(1).system = 2;
  output_select(1).output = 1;
  sysCl = mimoFeedback(sys,observer,[],[],[],output_select);
end

end