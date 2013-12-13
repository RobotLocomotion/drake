function testQP

w = warning('off','optim:quadprog:SwitchToMedScale');

compareQPSolvers(eye(2),zeros(2,1));
compareQPSolvers(1,1,1,50,[],[],-10,10);
compareQPSolvers(1,1,[],[],[],[],5,10);

warning(w);

end



