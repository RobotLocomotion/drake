function [Kp,Kd] = getPDGains(manip)

%NOTEST

B = manip.getB();
idx = B'*(1:manip.getNumStates()/2)';

fr=CoordinateFrame('q_d',length(idx),'d',{manip.getStateFrame.coordinates{idx}});
Kp = Point(fr);
Kd = Point(fr);

Kp.l_leg_uhz = 5.0;
Kp.l_leg_mhx = 100.0;
Kp.l_leg_lhy = 2000.0;
Kp.l_leg_kny = 1000.0;
Kp.l_leg_uay = 900.0;
Kp.l_leg_lax = 300.0;
Kp.r_leg_uhz = 5.0;
Kp.r_leg_mhx = 100.0;
Kp.r_leg_lhy = 2000.0;
Kp.r_leg_kny = 1000.0;
Kp.r_leg_uay = 900.0;
Kp.r_leg_lax = 300.0;
Kp.l_arm_usy = 500.0;
Kp.l_arm_shx = 200.0;
Kp.l_arm_ely = 50.0;
Kp.l_arm_elx = 50.0;
Kp.l_arm_uwy = 1.0;
Kp.l_arm_mwx = 2.0;
Kp.r_arm_usy = 500.0;
Kp.r_arm_shx = 200.0;
Kp.r_arm_ely = 50.0;
Kp.r_arm_elx = 50.0;
Kp.r_arm_uwy = 1.0;
Kp.r_arm_mwx = 2.0;
Kp.neck_ay = 20.0;
Kp.back_lbz = 20.0;
Kp.back_mby = 2000.0;
Kp.back_ubx = 800.0;

Kd.l_leg_uhz = 0.01;
Kd.l_leg_mhx = 1.0;
Kd.l_leg_lhy = 10.0;
Kd.l_leg_kny = 10.0;
Kd.l_leg_uay = 5.0;
Kd.l_leg_lax = 2.0;
Kd.r_leg_uhz = 0.01;
Kd.r_leg_mhx = 1.0;
Kd.r_leg_lhy = 10.0;
Kd.r_leg_kny = 10.0;
Kd.r_leg_uay = 5.0;
Kd.r_leg_lax = 2.0;
Kd.l_arm_usy = 3.0;
Kd.l_arm_shx = 20.0;
Kd.l_arm_ely = 3.0;
Kd.l_arm_elx = 3.0;
Kd.l_arm_uwy = 0.1;
Kd.l_arm_mwx = 0.2;
Kd.r_arm_usy = 3.0;
Kd.r_arm_shx = 20.0;
Kd.r_arm_ely = 3.0;
Kd.r_arm_elx = 3.0;
Kd.r_arm_uwy = 0.1;
Kd.r_arm_mwx = 0.2;
Kd.neck_ay = 1.0;
Kd.back_lbz = 0.1;
Kd.back_mby = 2.0;
Kd.back_ubx = 1.0;

Kp = diag(double(Kp));
Kd = diag(double(Kd));

end