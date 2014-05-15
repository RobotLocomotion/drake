function neighbors = mesh_neighbors(vertices, faces)
% Compute the neighbors (faces which share an edge) for each face in the
% mesh.
%
% @param vertices a [3 x nverts] matrix of vertex points
% @param faces a [3 x nfaces] matrix of faces. Each column of faces is a
%              triplet of indices into the columns of vertices.
% @retval neighbors a [3 x nfaces] matrix of neighbors. Each column
%                   neighbors(:,j) is the indices of the faces which share an 
%                   edge with the face given by faces(:,j). Missing
%                   neighbors are given by 0 entries.
%
% Additionally, the order of the neighbors matrix indicates which vertices
% are shared between adjacent faces. So any face 
%     faces(:,j) 
% and neighbor 
%     faces(:,neighbors(k,j))
% share the two vertices located at
%     vertices(:,[faces(k,j), faces(mod(k,3)+1,j)])
% which will be among the three vertices corresponding to the neighboring face:
%     vertices(:,[faces(:,neighbors(k,j))])

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
  
  for k = 1:3
    k1 = k;
    k2 = mod(k, 3) + 1;
    matches = find(masks{k1} & masks{k2});
    assert(ismember(j, matches));
    assert(length(matches) <= 2, 'Too many neighbors for this face');
    if length(matches) > 1
      neighbor = matches(matches ~= j);
      neighbors(k,j) = neighbor;
    end
  end
end

    