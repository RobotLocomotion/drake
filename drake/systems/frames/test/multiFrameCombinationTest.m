function multiFrameCombinationTest

fr1 = CoordinateFrame('frame1',2,'a');
fr2 = CoordinateFrame('frame2',2,'b');
fr3 = CoordinateFrame('frame3',1,'c');

sys1 = LinearSystem(eye(3),eye(3),[],[],[eye(3);zeros(2,3)],[zeros(5,3)]);
sys1 = setInputFrame(sys1,MultiCoordinateFrame({fr1,fr3}));
sys1 = setOutputFrame(sys1,MultiCoordinateFrame({fr1,fr2,fr3}));

sys2 = LinearSystem([],[],[],[],[],[eye(5)]);
sys2 = setInputFrame(sys2,MultiCoordinateFrame({fr3,fr2,fr1}));
sys2 = setOutputFrame(sys2,MultiCoordinateFrame({fr3,fr1,fr2}));

sys=mimoCascade(sys1,sys2);
%open_system(getModel(sys));

output_select(1).system=2;
output_select(1).output=1;
output_select(2).system=2;
output_select(2).output=2;
sys=mimoCascade(sys2,sys1,[],[],output_select);
%open_system(getModel(sys));

sys=mimoFeedback(sys1,sys2);
%open_system(getModel(sys));