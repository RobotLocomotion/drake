function testRandomTree

T = struct('parent',zeros(1,1000),'node',zeros(2,1000));  % pre-allocate memory for the "tree"
for i=2:size(T.parent,2)
  T.parent(i) = randi(i-1);
  x_rand = T.node(:,T.parent(i));
  u_rand = 2*rand(2,1)-1;
  x_new = x_rand+u_rand;
  if (15<=x_new(1) && x_new(1)<=20 && 15<=x_new(2) && x_new(2)<=20)
    disp('Success!'); break;
  end
  T.node(:,i) = x_new;
end
clf;
line([T.node(1,T.parent(2:end));T.node(1,2:end)],[T.node(2,T.parent(2:end));T.node(2,2:end)],'Color','k');
patch([15,15,20,20],[15,20,20,15],'g')
axis([-10,25,-10,25]);
  
