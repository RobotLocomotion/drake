function [K, S] = ZMPCachedLQR(h_over_g, Qy)
% NOTEST

MAX_CACHE_ENTRIES = 1000;

key = [sprintf('hg %f ', h_over_g), 'Qy ', sprintf('%f', Qy)];

persistent lqr_solution_cache
if isempty(lqr_solution_cache)
  fname = fullfile(getDrakePath(), 'examples', 'ZMP', 'data', 'lqr_solution_cache.mat');
  if exist(fname, 'file')
    S = load(fname, 'lqr_solution_cache');
    lqr_solution_cache = S.lqr_solution_cache;
  else
    lqr_solution_cache = containers.Map('KeyType', 'char', 'ValueType', 'any');
  end
end

cache_dirty = false;
if lqr_solution_cache.isKey(key)
  value = lqr_solution_cache(key);
  K = value{1};
  S = value{2};
else
  [A, B, C, D, Q, R, Q1, R1, N] = LinearInvertedPendulum.setupLinearSystem(h_over_g, Qy);
  [K, S] = lqr(A, B, Q1, R1, N);
  lqr_solution_cache(key) = {K, S};
  cache_dirty = true;
end

if length(lqr_solution_cache) > MAX_CACHE_ENTRIES;
  keys = lqr_solution_cache.keys();
  lqr_solution_cache.remove(keys(1:floor(end/2)));
  cache_dirty = true;
end

if cache_dirty
  fname = fullfile(getDrakePath(), 'examples', 'ZMP', 'data', 'lqr_solution_cache.mat');
  save(fname, 'lqr_solution_cache');
end

