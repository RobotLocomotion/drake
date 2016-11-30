function moveSpatialMassMatrixTest
%Checks the moveSpatialMassMatrix function with a number of known cases

disp('Checking RigidBodyAddedMass.moveSpatialMassMatrix function');
f = @RigidBodyAddedMass.moveSpatialMassMatrix;
c = 1;
a = c/2;
cm = [c/4;0;0];
Icm = diag([0 0 1/8*pi*a^4]);
m = diag([0 pi*a^2 0]);
Z = zeros(3);

I_example = [Icm Z; Z m];

m22 = pi*a^2;
m26 = 1/2*pi*a^3;
m66 = 1/8*pi*a^4+(a/2)^2*m22;
I_move_true = [diag([0 m22 0]) [0 0 0; 0 0 m26; 0 0 0];...
    [0 0 0; 0 0 0; 0 m26 0] diag([0 0 m66])];

disp('Check1')
I_move_true = I_move_true([4:6 1:3],[4:6 1:3]);
I_move = f(cm,I_example);
valuecheck(I_move,I_move_true);

disp('Check2')
I_magic = magic(6)+magic(6)';
I_moveThereAndBack = f(-cm,f(cm,I_magic));
valuecheck(I_magic,I_moveThereAndBack);

disp('Check3')
I_moveByThree = f(3*cm,I_magic);
I_moveThreeTimes = f(cm,f(cm,f(cm,I_magic)));
valuecheck(I_moveByThree,I_moveThreeTimes);

end
