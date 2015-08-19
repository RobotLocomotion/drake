function deepMultiFrameTest

lfr1 = CoordinateFrame('leafframe1',2,'a');
lfr2 = CoordinateFrame('leafframe2',2,'b');
lfr3 = CoordinateFrame('leafframe3',2,'c');
lfr4 = CoordinateFrame('leafframe4',2,'d');
sfr1 = MultiCoordinateFrame({lfr1, lfr2});
sfr2 = MultiCoordinateFrame({lfr3, sfr1});
fr1 = MultiCoordinateFrame({sfr2, lfr4});

% Test ability to pull out deeply embedded coordinate frames
out_lfr1 = fr1.getFrameByNameRecursive('leafframe1');
if (out_lfr1 ~= lfr1)
  error('Returned incorrect subrame!');
end
out_lfr1_fr = fr1.getFrameNumByNameRecursive('leafframe1');
sizecheck(out_lfr1_fr, [1 3]); valuecheck(out_lfr1_fr, [1 2 1]);

out_lfr2 = fr1.getFrameByNameRecursive('leafframe2');
if (out_lfr2 ~= lfr2)
  error('Returned incorrect subrame!');
end
out_lfr2_fr = fr1.getFrameNumByNameRecursive('leafframe2');
sizecheck(out_lfr2_fr, [1 3]); valuecheck(out_lfr2_fr,[1 2 2]);

out_lfr3 = fr1.getFrameByNameRecursive('leafframe3');
if (out_lfr3 ~= lfr3)
  error('Returned incorrect subrame!');
end
out_lfr3_fr = fr1.getFrameNumByNameRecursive('leafframe3');
sizecheck(out_lfr3_fr, [1 2]); valuecheck(out_lfr3_fr, [1 1]);

out_lfr4 = fr1.getFrameByNameRecursive('leafframe4');
if (out_lfr4 ~= lfr4)
  error('Returned incorrect subrame!');
end
out_lfr4_fr = fr1.getFrameNumByNameRecursive('leafframe4');
sizecheck(out_lfr4_fr, [1 1]); valuecheck(out_lfr4_fr, [2]);

% And see if we can get the sub-multicoordinate frames too
out_sfr1 = fr1.getFrameByNameRecursive('leafframe1+leafframe2');
if (out_sfr1 ~= sfr1)
  error('Returned incorrect subrame!');
end
out_sfr1_fr = fr1.getFrameNumByNameRecursive('leafframe1+leafframe2');
sizecheck(out_sfr1_fr, [1 2]); valuecheck(out_sfr1_fr, [1 2]);

out_sfr2 = fr1.getFrameByNameRecursive('leafframe3+leafframe1+leafframe2');
if (out_sfr2 ~= sfr2)
  error('Returned incorrect subrame!');
end
out_sfr2_fr = fr1.getFrameNumByNameRecursive('leafframe3+leafframe1+leafframe2');
sizecheck(out_sfr2_fr, [1 1]); valuecheck(out_sfr2_fr, [1]);