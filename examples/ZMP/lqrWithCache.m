function [K, S] = lqrWithCache(A, B, Q, R, N)

MAX_CACHE_ENTRIES = 1000;

key = sprintf('%f ', A, B, Q, R, N);

fname = fullfile(getDrakePath(), 'examples', 'ZMP', 'data', 'lqr_solution_cache.mat');
persistent lqr_solution_cache
if isempty(lqr_solution_cache)
  if exist(fname, 'file')
    S = load(fname, 'lqr_solution_cache');
    lqr_solution_cache = S.lqr_solution_cache;
  else
    lqr_solution_cache = containers.Map('KeyType', 'char', 'ValueType', 'any');
  end
end

if lqr_solution_cache.isKey(key)
  value = lqr_solution_cache(key);
  K = value{1};
  S = value{2};
else
  [K, S] = lqr(A, B, Q, R, N);
  lqr_solution_cache(key) = {K, S};
end

if length(lqr_solution_cache) > MAX_CACHE_ENTRIES;
  keys = lqr_solution_cache.keys();
  lqr_solution_cache.remove(keys(1:floor(end/2)));
end

save(fname, 'lqr_solution_cache');

