load atlas_hybrid_dircol_good_2
xtraj_=xtraj{1}.append(xtraj{2}).append(xtraj{3}).append(xtraj{4}).append(xtraj{5});;
utraj_=utraj{1}.append(utraj{2}).append(utraj{3}).append(utraj{4}).append(utraj{5});;
for i=1:length(xtraj),
  R{i} = zeros(16,2*length(contact_seq{i}));
  R{i}(contact_seq{i}*4-3,2:2:end) = eye(length(contact_seq{i}));
  ltraj_full{i} = R{i}*ltraj{i};
end
ltraj_=ltraj_full{1}.append(ltraj_full{2}).append(ltraj_full{3}).append(ltraj_full{4});
solveLQR(p,xtraj_,utraj_,ltraj_);