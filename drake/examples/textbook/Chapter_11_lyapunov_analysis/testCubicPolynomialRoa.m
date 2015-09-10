function testCubicPolynomialRoa

  cd(fullfile(getDrakePath,'examples'));

  % create a new CubicPolynomialExample object
  p = CubicPolynomialExample();

  % compute region of attraction
  % the levelset V<1 is the region of attraction
  V=regionOfAttraction(p,Point(p.getStateFrame,0));

  % display the polynomial representation of V that results
  display(V.getPoly);
  
