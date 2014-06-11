function linearize_test

sys = SimulinkModel('linear_plant');

[A,B,C,D] = linearize(sys,0,zeros(2,1),0);

if (any(any(abs([A,B,C,D] - [[1,2;3,4],[5;6],eye(2),zeros(2,1)])>1e-9)))
  error('something''s not right');
end