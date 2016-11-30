function testDcross
  for i = 1:100
    r1 = rand(3,1);
    r2 = rand(3,1);
    [~,dr1crossr2_taylorvar] = geval(@cross,r1,r2,struct('grad_method','taylorvar'));
    dr1crossr2 = dcross(r1,r2, [eye(3), zeros(3)], [zeros(3), eye(3)]);
    dr1crossr2_error = dr1crossr2 - dr1crossr2_taylorvar;
    assert(max(abs(dr1crossr2_error(:))) < eps)
  end
end
