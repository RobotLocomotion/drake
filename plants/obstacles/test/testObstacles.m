function testObstacles

close all

f = ObstacleField
f.GenerateRandomObstacles


xmin = -2;
xmax = 12;

[xa ya] = meshgrid(linspace(xmin, xmax,200));

figure(25)
clf
axis([xmin xmax xmin xmax]);
xlabel('X')
ylabel('Y')
hold on

for (q=1:8)%f.number_of_obstacles)
  disp(q)
  x = f.obstacles{q}.getConstraints;
  func = x.x.c;
  
  for (i=1:length(xa))
    for (j=1:length(ya))

      val(j,i) = func(xa(1,i), ya(j,1));
    end
  end
  h = surf(xa, ya, val);
  
  set(h, 'edgecolor','none')
  
  f.obstacles{q}.draw()
end




