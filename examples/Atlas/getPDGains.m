function [Kp,Kd] = getPDGains(r)

%NOTEST

B = r.getB();
idx = B'*(1:r.getNumStates()/2)';

fr=CoordinateFrame('q_d',length(idx),'d',{r.getStateFrame.coordinates{idx}});
Kp = Point(fr);
Kd = Point(fr);

Kp.l_leg_uhz = 5.0;
Kp.l_leg_mhx = 200.0;
Kp.l_leg_lhy = 2000.0;
Kp.l_leg_kny = 1000.0;
Kp.l_leg_uay = 900.0;
Kp.l_leg_lax = 500.0;
Kp.r_leg_uhz = Kp.l_leg_uhz;
Kp.r_leg_mhx = Kp.l_leg_mhx;
Kp.r_leg_lhy = Kp.l_leg_lhy;
Kp.r_leg_kny = Kp.l_leg_kny;
Kp.r_leg_uay = Kp.l_leg_uay;
Kp.r_leg_lax = Kp.l_leg_lax;
Kp.l_arm_usy = 500.0;
Kp.l_arm_shx = 200.0;
Kp.l_arm_ely = 50.0;
Kp.l_arm_elx = 50.0;
Kp.l_arm_uwy = 10.0;
Kp.l_arm_mwx = 10.0;
Kp.r_arm_usy = Kp.l_arm_usy;
Kp.r_arm_shx = Kp.l_arm_shx;
Kp.r_arm_ely = Kp.l_arm_ely;
Kp.r_arm_elx = Kp.l_arm_elx;
Kp.r_arm_uwy = Kp.l_arm_uwy;
Kp.r_arm_mwx = Kp.l_arm_mwx;
Kp.neck_ay = 20.0;
Kp.back_lbz = 200.0;
Kp.back_mby = 2000.0;
Kp.back_ubx = 800.0;

Kd.l_leg_uhz = 0.01;
Kd.l_leg_mhx = 3.0;
Kd.l_leg_lhy = 16.0;
Kd.l_leg_kny = 16.0;
Kd.l_leg_uay = 18.0;
Kd.l_leg_lax = 5.0;
Kd.r_leg_uhz = Kd.l_leg_uhz;
Kd.r_leg_mhx = Kd.l_leg_mhx;
Kd.r_leg_lhy = Kd.l_leg_lhy;
Kd.r_leg_kny = Kd.l_leg_kny;
Kd.r_leg_uay = Kd.l_leg_uay;
Kd.r_leg_lax = Kd.l_leg_lax;
Kd.l_arm_usy = 3.0;
Kd.l_arm_shx = 20.0;
Kd.l_arm_ely = 3.0;
Kd.l_arm_elx = 3.0;
Kd.l_arm_uwy = 0.25;
Kd.l_arm_mwx = 0.25;
Kd.r_arm_usy = Kd.l_arm_usy;
Kd.r_arm_shx = Kd.l_arm_shx;
Kd.r_arm_ely = Kd.l_arm_ely;
Kd.r_arm_elx = Kd.l_arm_elx;
Kd.r_arm_uwy = Kd.l_arm_uwy;
Kd.r_arm_mwx = Kd.l_arm_mwx;
Kd.neck_ay = 1.0;
Kd.back_lbz = 8.0;
Kd.back_mby = 16.0;
Kd.back_ubx = 6.0;

Kp = diag(double(Kp))*1.25;
Kd = diag(double(Kd))*4.5;

end