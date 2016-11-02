function testDDcross
  for i = 1:100
    r1 = rand(3,1);
    r2 = rand(3,1);
    [~,ddr1crossr2_taylorvar] = geval(@dcross,r1,r2,[eye(3), zeros(3)], [zeros(3), eye(3)],struct('grad_method','taylorvar'));
    ddr1crossr2 = ddcross(r1,r2, [eye(3), zeros(3)], [zeros(3), eye(3)]);
    ddr1crossr2_error = ddr1crossr2 - ddr1crossr2_taylorvar;
    assert(max(abs(ddr1crossr2_error(:))) < eps)
  end
end
