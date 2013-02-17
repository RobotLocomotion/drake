function massSpringDamper  

r = RigidBodyManipulator('MassSpringDamper.urdf');

x0 = [5;0];%randn(2,1);
xtraj = simulate(r,[0 5],x0);
fnplt(xtraj,1);
%      v = r.constructVisualizer();
%      v.playback(xtraj);

end
  
  


