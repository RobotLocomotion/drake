function testVerticalAndHorizontalObstacles

tmp = addpathTemporary(fullfile(pwd,'..'));

%% Test edge cases for vertical and horizontal obstacle borders
obs = PolygonalObstacle2D([0,0,1,1], [0,1,1,0]);

con = obs.getConstraints();
func = con.x.c;

% moving parallel along bottom edge should not change distance
valuecheck(func(0.25, -0.1), func(0.5, -0.1));

% moving parallel along top edge should not change distance
valuecheck(func(0.25, 1.1), func(0.5, 1.1));

% moving parallel along left edge should not change distance
valuecheck(func(-0.1, 0.25), func(-0.1, 0.25));

% moving parallel along right edge should not change distance
valuecheck(func(1.1, 0.25), func(1.1, 0.5));