function testInvertFlatOutputs

p = addpathTemporary(fullfile(pwd,'..'));

[utraj,xtraj,~,plant] = runDircol;

ytraj = xtraj([1:3,6]);
ytraj = setOutputFrame(ytraj,DifferentiallyFlatOutputFrame);

[xtraj_from_flat,utraj_from_flat] = invertFlatOutputs(plant,ytraj);

% todo: test the remaining coordinates here
% todo: improve tolerance (by finishing derivatives, etc)
valuecheck(xtraj_from_flat(1:6),xtraj(1:6),.1);
