function mass = totalMass(model, robotnum)
mass = 0;
for i = 1 : length(model.body)
  if(any(model.body(i).robotnum == robotnum))
    mass = mass + model.body(i).mass;
  end
end
end