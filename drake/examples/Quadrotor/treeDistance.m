function [c, dc] = treeDistance(q)

tree_pos = [[.495;2.3], [2.2;4.5], [-1.85;5.2], [0;5.8], [.25;7.5], [-1.35;8.5]];

    c = zeros(size(tree_pos,2),1);
    dc = zeros(size(tree_pos,2),2);
    for k = 1:length(c)
        c(k) = (q(1:2)-tree_pos(1:2,k))'*(q(1:2)-tree_pos(1:2,k));
        dc(k,:) = 2*(q(1:2)-tree_pos(1:2,k))';
    end
end