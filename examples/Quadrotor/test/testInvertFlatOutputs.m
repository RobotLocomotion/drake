function testInvertFlatOutputs

p = addpathTemporary(fullfile(pwd,'..'));

[utraj,xtraj,~,plant] = runDircol;

ytraj = xtraj([1:3,6]);
ytraj = setOutputFrame(ytraj,DifferentiallyFlatOutputFrame);

[xtraj_from_flat,utraj_from_flat] = invertFlatOutputs(plant,ytraj);

% todo: improve tolerance (by outputting polynomials, etc)
valuecheck(xtraj_from_flat(1:6),xtraj(1:6),.1);
% note: velocities are implemented, but do not pass a numeric test because 
% ytraj is not defined to high enough order
