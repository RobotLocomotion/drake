function approx_demo

clf;

f = inline('sin(x).^2  + tanh(x)');

x = 0:0.01:10;
y = f(x);


xdata = 10*rand(100,1);
ydata = f(xdata)+0.1*randn(100,1);

hold on;
plot(x,y);

pause;

plot(xdata,ydata,'r.');

pause;

% radial basis functions:
phi = @(x,m) exp(-(x-m).^2);

% "fourier" basis
%phi = @(x,m) sin(m*x/(2*pi));

%% polynomial basis
%%phi = @(x,m) x.^m;

% barycentric (linear) interpolation
%phi = @(x,m) max(1-abs(m-x),0);

% means of basis function
m = 0:1:10;  
%m = 0:0.1:10;   % shows some overfitting for rbfs

for i=1:length(m)
  yf = phi(x,m(i));
  P(:,i) = phi(xdata,m(i));
  plot(x,yf,'g','LineWidth',2);
end

w = P\ydata;

pause;

clf;
hold on;
plot(x,y);
plot(xdata,ydata,'r.');

yhat = zeros(size(y));
for i=1:length(m)
  yf = w(i)*phi(x,m(i));
  yhat = yhat + yf;
  plot(x,yf,'g','LineWidth',2);
end

axis([0,10,-0.5,2.5]);

pause;
plot(x,yhat,'k--','LineWidth',2);