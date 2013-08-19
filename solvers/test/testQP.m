function testQP

compareSolvers(eye(2),zeros(2,1));
compareSolvers(1,1,[],[],1,50,-10,10);
compareSolvers(1,1,[],[],[],[],5,10);

end



