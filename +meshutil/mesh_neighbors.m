function neighbors = mesh_neighbors(vertices, faces)

neighbors = zeros(3, size(faces, 2));
for j = 1:size(faces, 2)
  pts = faces(:,j); 
  masks = {zeros(1, size(faces, 2)), zeros(1, size(faces, 2)), zeros(1, size(faces, 2))};

  for k = 1:3
    matching_vertices = find(all(bsxfun(@eq, vertices, vertices(:,pts(k))), 1));
     for i = 1:length(matching_vertices)
      masks{k} = masks{k} | any(faces == matching_vertices(i), 1);
     end
  end
  
%   [~, c1] = find(faces == pts(1));
%   [~, c2] = find(faces == pts(2));
%   [~, c3] = find(faces == pts(3));
%   cs = {c1, c2, c3};
  
  for k = 1:3
    k1 = k;
    k2 = mod(k, 3) + 1;
    matches = find(masks{k1} & masks{k2});
%     matches = intersect(cs{k1}, cs{k2});
    assert(ismember(j, matches));
    assert(length(matches) <= 2, 'Too many neighbors for this face');
    if length(matches) > 1
      neighbor = matches(matches ~= j);
      neighbors(k,j) = neighbor;
    end
  end
end

    