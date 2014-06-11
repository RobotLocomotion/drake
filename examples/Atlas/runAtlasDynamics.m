function runAtlasDynamics
% Simulate the (passive) dynamics of the Atlas model 

% Load the model with a floating base
options.floating = true;
options.dt = 0.001;
options.terrain = RigidBodyFlatTerrain;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

% Initialize the viewer
v = r.constructVisualizer;
v.display_dt = 0.01;

% Compute a feasible set of initial conditions for the simulation (e.g. no
% penetration)
x0 = Point(r.getStateFrame);
x0 = resolveConstraints(r,x0);

if (1) % Run simulation, then play it back at realtime speed
  tic;
  xtraj = simulate(r,[0 2],x0);
  toc;
  playback(v,xtraj,struct('slider',true));
else % View the simulation as it is being computed
  % we are knowingly breaking out to a simulink model with the cascade on the following line.
  warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  
  sys = cascade(r,v);
  simulate(sys,[0 2],x0);
end

end
