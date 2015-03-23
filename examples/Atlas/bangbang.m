function bangbang()
% ric = A exp(B*t) + C;
% ric_ = A + C + AB * t + AB^2 * t^2 / 2 + AB^3 * t^3 / 6

A = rand();
B = 0.4;
C = randn();

  function r = r_ic(t)
    r = A * exp(B*t) + C;
  end

n_taylor = 5;
p_ic = zeros(n_taylor+1, 1);
for j = 1:n_taylor+1
  p_ic(j) = A * B^(n_taylor+1-j);
  if j == n_taylor + 1
    p_ic(j) = p_ic(j) + C;
  end
  p_ic(j) = p_ic(j) / factorial(n_taylor+1-j);
end

  function r = r_ic_approx(t)
    mon = zeros(n_taylor+1, length(t));
    for j = 1:n_taylor+1
      mon(j,:) = t.^(n_taylor+1-j);
    end
    r = p_ic' * mon;
  end

l0 = randn()

ld0 = randn()


  function r = r_foot(t)
    r = l0 + 0.5 * ld0 * t + 1/4 * a * t.^2 - 1/4 * ld0^2/a;
  end

a = 1;

figure(5)
clf
hold on
for j = 1:50
  tf = abs(ld0)/a + 5 * rand();
  ts = 0.5 * (tf - ld0/a);
  ls = l0 + ld0 * ts + 0.5 * a * ts^2;
  lds = ld0 + a * ts;
  
  tt = linspace(0, ts);
  plot(tt, l0 + ld0 * tt + 0.5 * a * tt.^2)
  
  tt = linspace(ts, tf);
  plot(tt, ls + lds * (tt - ts) + 0.5 * -1 * a * (tt-ts).^2);
  
  plot(tf, r_foot(tf), 'ro');
%   plot(tf, l0 + 0.5 * ld0 * tf + 1/4 * a * tf^2 - 1/4 * ld0^2 / a, 'ro');
end
  
tt = linspace(0, 10);
plot(tt, r_ic(tt), 'g-');
% plot(tt, A * exp(B .* tt) + C, 'g-')

tf = linspace(0, 10);
figure(6);
clf
hold on
plot(tf, r_foot(tf), 'r-');
% plot(tf, l0 + 0.5 * ld0 * tf + 1/4 * a * tf.^2 - 1/4 * ld0^2 / a, 'r-')
plot(tf, 0.5 * (tf - ld0 / a), 'm-')
plot(tf, r_ic(tf), 'g-');
plot(tf, r_ic_approx(tf), 'g--');
% plot(tt, A * exp(B .* tt) + C, 'g-')
% plot(tt, A + C + A*B * tt + A * B^2 * tt.^2 + A*B^3 * tt.^3 + A*B^4 * tt.^4, 'g--')

% p_ic = [A * B^4;
%         A * B^3;
%         A * B^2;
%         A * B;
%         A + C];
p_f = [zeros(n_taylor-2, 1);
       1/4 * a;
       1/2 * ld0;
       l0 - 1/4 * ld0^2/a];

tic(); t_int = roots(p_ic - p_f); toc();
mask = false(size(t_int));
for j = 1:size(t_int)
  mask(j) = isreal(t_int(j)) && t_int(j) > 0;
end
t_int = t_int(mask)

for j = 1:length(t_int)
  plot(t_int(j), r_foot(t_int(j)), 'ro');
end

end
