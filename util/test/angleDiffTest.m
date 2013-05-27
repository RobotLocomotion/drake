function angleDiffTest()

  for j = 1:1000
    phi1 = random('unif', -3*pi, 3*pi,3,1);
    theta = random('unif', -pi, pi,3,1);
    phi2 = phi1 + theta;
    d = angleDiff(phi1, phi2);
    uw = unwrap([phi1, phi2],[],2);
    duw = uw(:,2)-uw(:,1);
    
    try
      valuecheck(d - theta,0);
      valuecheck(duw - theta,0);
    catch err
      phi1
      phi2
      theta
      d
      uw
      duw
      disp(err);
      error('Failed angleDiffTest');
    end
     
  end