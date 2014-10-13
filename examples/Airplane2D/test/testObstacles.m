function testObstacles

tmp = addpathTemporary(fullfile(pwd,'..'));

f = ObstacleField
f = f.GenerateRandomObstacles();


xmin = -2;
xmax = 12;

[xa ya] = meshgrid(linspace(xmin, xmax,20));

figure(25)
clf
axis([xmin xmax xmin xmax]);
xlabel('X')
ylabel('Y')
hold on

for (q=1:f.number_of_obstacles)
  disp(q)
  x = f.obstacles{q}.getConstraints;
  func = x.x.c;
  val = arrayfun(func, xa, ya);

  h = surf(xa, ya, val);

  set(h, 'edgecolor','none')

  f.obstacles{q}.draw()
end




