function foh_test

t = 1:10;
ts = 1:.1:10;

figure(1)
y = randn(1,10);
ypp = foh(t,y);
plot(ts,ppval(ypp,ts),'b-',t,y,'r.');

figure(2)
y = randn(2,2,10);
ypp = foh(t,y);
ys = ppval(ypp,ts);
plot(ts,reshape(ys,[],length(ts)),'b-',t,reshape(y,[],length(t)),'r.');



