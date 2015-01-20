function PassByValueMapTest()
  handle_map = containers.Map();
  value_map = PassByValueMap();

  % Check that the empty maps are equal
  assert(isequal(handle_map, value_map.getCopyOfInternalMap()))

  % Check that isempty works for empty map
  assert(isempty(value_map));

  % Check simple assignment
  handle_map('abc') = 1;
  value_map('abc') = 1;
  value_map2 = value_map;

  handle_map('ghi') = 2;
  value_map('ghi') = 2;
  assert(isequal(handle_map, value_map.getCopyOfInternalMap()))

  % Check that modifying value_map didn't modify value_map 2
  assert(value_map.Count == 2);
  assert(value_map2.Count == 1);

  % Test methods
  % iskey
  assert(value_map.isKey('abc'));
  assert(~value_map.isKey('def'));

  % keys
  assert(isequal(handle_map.keys, value_map.keys));

  % values
  assert(isequal(handle_map.values, value_map.values));

  % size
  assert(isequal(handle_map.size, value_map.size));

  % length
  assert(isequal(handle_map.length, value_map.length));

  % isempty
  assert(isequal(handle_map.isempty, value_map.isempty));

  % KeyType
  assert(isequal(handle_map.KeyType, value_map.KeyType));

  % ValueType
  assert(isequal(handle_map.ValueType, value_map.ValueType));

  % remove
  value_map3 = value_map;
  value_map.remove('abc');
  assert(isequal(value_map, value_map3));

  value_map = value_map.remove('ghi');
  assert(isequal(value_map, value_map2));
end
