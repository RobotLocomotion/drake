function runAtlasDynamics
% just runs it as a passive system 

% just runs it as a passive system for now
options.floating = true;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

v = r.constructVisualizer;
v.display_dt = 0.01;

x0 = Point(r.getStateFrame);
x0 = resolveConstraints(r,x0);

if (1)
  % Run simulation, then play it back at realtime speed
  tic;
  xtraj = simulate(r,[0 2],x0);
  toc;
  playback(v,xtraj,struct('slider',true));
else    
  % we are knowingly breaking out to a simulink model with the cascade on the following line.
  warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  
  sys = cascade(r,v);
  simulate(sys,[0 2],x0);
end

end