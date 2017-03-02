function testDDcross
  for i = 1:100
    r1 = rand(3,1);
    r2 = rand(3,1);
    dr1 = [1;0;0];
    dr2 = [1;0;0];
    [~,ddr1crossr2_taylorvar] = geval(@dcross,r1,r2,dr1,dr2,struct('grad_method','numerical'));
    ddr1crossr2 = ddcross(r1,r2, dr1, dr2, [eye(3), zeros(3)], [zeros(3), eye(3)]);
    ddr1crossr2_error = ddr1crossr2 - ddr1crossr2_taylorvar;
    assert(max(abs(ddr1crossr2_error(:))) < eps)
  end
end
