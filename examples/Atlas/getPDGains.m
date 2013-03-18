function [Kp,Kd] = getPDGains(r,mode)
%NOTEST

if nargin<2
  mode = 'default';
end

fr = getInputFrame(r);
Kp = Point(fr);
Kd = Point(fr);

if strcmpi(mode,'default')
  Kp.l_arm_usy = 625.0; %%%%%
  Kp.l_arm_shx = 3000.0; %%%%%
  Kp.l_arm_ely = 650.0; %%%%%
  Kp.l_arm_elx = 1600.0; %%%%%s
  Kp.l_arm_uwy = 25.0; %%%%%
  Kp.l_arm_mwx = 375.0; %%%%%
  Kp.l_leg_uhz = 800.0; 
  Kp.l_leg_mhx = 6000.0; 
  Kp.l_leg_lhy = 2200.0; 
  Kp.l_leg_kny = 3500.0; 
  Kp.l_leg_uay = 3300.0;
  Kp.l_leg_lax = 3300.0;
  Kp.neck_ay = 40.0; %%%%%
  Kp.back_lbz = 4500.0; % 
  Kp.back_mby = 6000.0; %
  Kp.back_ubx = 2350.0; %

  Kd.l_arm_usy = 38.0; %%%%%
  Kd.l_arm_shx = 75.0; %%%%%
  Kd.l_arm_ely = 30.0; %%%%%
  Kd.l_arm_elx = 60.0; %%%%%
  Kd.l_arm_uwy = 1.0; %%%%%
  Kd.l_arm_mwx = 10.0; %%%%%
  Kd.l_leg_uhz = 55.0; 
  Kd.l_leg_mhx = 95.0; 
  Kd.l_leg_lhy = 50.0; 
  Kd.l_leg_kny = 85.0; 
  Kd.l_leg_uay = 90.0;
  Kd.l_leg_lax = 90.0;  
  Kd.neck_ay = 4.0; %%%%%
  Kd.back_lbz = 50.0; %
  Kd.back_mby = 65.0; %
  Kd.back_ubx = 105.0; %

elseif strcmpi(mode,'pinned')
  Kp.l_arm_usy = 625.0; %%%%%
  Kp.l_arm_shx = 3000.0; %%%%%
  Kp.l_arm_ely = 650.0; %%%%%
  Kp.l_arm_elx = 1600.0; %%%%%
  Kp.l_arm_uwy = 25.0; %%%%%
  Kp.l_arm_mwx = 375.0; %%%%%
  Kp.l_leg_uhz = 250.0; %
  Kp.l_leg_mhx = 350.0; %
  Kp.l_leg_lhy = 400.0; %
  Kp.l_leg_kny = 400.0; %
  Kp.l_leg_uay = 45.0; %
  Kp.l_leg_lax = 45.0; %
  Kp.neck_ay = 40.0; %%%%%
  Kp.back_lbz = 1750.0; % 
  Kp.back_mby = 4200.0; %
  Kp.back_ubx = 2200.0; %

  Kd.l_arm_usy = 38.0; %%%%%
  Kd.l_arm_shx = 75.0; %%%%%
  Kd.l_arm_ely = 30.0; %%%%%
  Kd.l_arm_elx = 60.0; %%%%%
  Kd.l_arm_uwy = 1.0; %%%%%
  Kd.l_arm_mwx = 10.0; %%%%%
  Kd.l_leg_uhz = 30.0; %
  Kd.l_leg_mhx = 60.0; %
  Kd.l_leg_lhy = 60.0; %
  Kd.l_leg_kny = 35.0; %
  Kd.l_leg_uay = 5.0; %
  Kd.l_leg_lax = 5.0; %
  Kd.neck_ay = 4.0; %%%%%
  Kd.back_lbz = 170.0; %
  Kd.back_mby = 300.0; %
  Kd.back_ubx = 250.0; %
  
elseif strcmpi(mode,'stance_leg')
  Kp.l_arm_usy = 625.0; %%%%%
  Kp.l_arm_shx = 3000.0; %%%%%
  Kp.l_arm_ely = 650.0; %%%%%
  Kp.l_arm_elx = 1600.0; %%%%%
  Kp.l_arm_uwy = 25.0; %%%%%
  Kp.l_arm_mwx = 375.0; %%%%%
  Kp.l_leg_uhz = 800.0; 
  Kp.l_leg_mhx = 6200.0; 
  Kp.l_leg_lhy = 2200.0; 
  Kp.l_leg_kny = 3500.0; 
  Kp.l_leg_uay = 3350.0;
  Kp.l_leg_lax = 3300.0;
  Kp.neck_ay = 40.0; %%%%%
  Kp.back_lbz = 4500.0; % 
  Kp.back_mby = 6000.0; %
  Kp.back_ubx = 2350.0; %

  Kd.l_arm_usy = 38.0; %%%%%
  Kd.l_arm_shx = 75.0; %%%%%
  Kd.l_arm_ely = 30.0; %%%%%
  Kd.l_arm_elx = 60.0; %%%%%
  Kd.l_arm_uwy = 1.0; %%%%%
  Kd.l_arm_mwx = 10.0; %%%%%
  Kd.l_leg_uhz = 55.0; 
  Kd.l_leg_mhx = 92.0; 
  Kd.l_leg_lhy = 50.0; 
  Kd.l_leg_kny = 85.0; 
  Kd.l_leg_uay = 100.0;
  Kd.l_leg_lax = 160.0;
  Kd.neck_ay = 4.0; %%%%%
  Kd.back_lbz = 50.0; %
  Kd.back_mby = 65.0; %
  Kd.back_ubx = 105.0; %
    
elseif strcmpi(mode,'gazebo')
  Kp.l_arm_usy = 2000.0; 
  Kp.l_arm_shx = 1000.0; 
  Kp.l_arm_ely = 200.0; 
  Kp.l_arm_elx = 200.0; 
  Kp.l_arm_uwy = 50.0; 
  Kp.l_arm_mwx = 100.0; 
  Kp.l_leg_uhz = 5.0; 
  Kp.l_leg_mhx = 100.0; 
  Kp.l_leg_lhy = 2000.0; 
  Kp.l_leg_kny = 1000.0; 
  Kp.l_leg_uay = 900.0;
  Kp.l_leg_lax = 300.0;
  Kp.neck_ay = 20.0; 
  Kp.back_lbz = 20.0;  
  Kp.back_mby = 4000.0; 
  Kp.back_ubx = 2000.0; 

  Kd.l_arm_usy = 3.0; 
  Kd.l_arm_shx = 10.0; 
  Kd.l_arm_ely = 3.0; 
  Kd.l_arm_elx = 3.0; 
  Kd.l_arm_uwy = 0.1; 
  Kd.l_arm_mwx = 0.2; 
  Kd.l_leg_uhz = 0.01; 
  Kd.l_leg_mhx = 1.0; 
  Kd.l_leg_lhy = 10.0; 
  Kd.l_leg_kny = 10.0; 
  Kd.l_leg_uay = 8.0;
  Kd.l_leg_lax = 2.0;
  Kd.neck_ay = 1.0; 
  Kd.back_lbz = 0.1; 
  Kd.back_mby = 2.0; 
  Kd.back_ubx = 1.0; 
  
elseif strcmpi(mode,'gazebo_old')
  Kp.l_arm_usy = 750.0; 
  Kp.l_arm_shx = 1300.0; 
  Kp.l_arm_ely = 800.0; 
  Kp.l_arm_elx = 850.0; 
  Kp.l_arm_uwy = 50.0; 
  Kp.l_arm_mwx = 300.0; 
  Kp.l_leg_uhz = 100.0; 
  Kp.l_leg_mhx = 920.0; 
  Kp.l_leg_lhy = 1550.0; 
  Kp.l_leg_kny = 1850.0; 
  Kp.l_leg_uay = 1900.0;
  Kp.l_leg_lax = 740.0;
  Kp.neck_ay = 50.0; 
  Kp.back_lbz = 800.0;  
  Kp.back_mby = 2100.0; 
  Kp.back_ubx = 800.0; 

  Kd.l_arm_usy = 65.0; 
  Kd.l_arm_shx = 85.0; 
  Kd.l_arm_ely = 55.0; 
  Kd.l_arm_elx = 65.0; 
  Kd.l_arm_uwy = 5.0; 
  Kd.l_arm_mwx = 20.0; 
  Kd.l_leg_uhz = 9.0; 
  Kd.l_leg_mhx = 21.0; 
  Kd.l_leg_lhy = 20.0; 
  Kd.l_leg_kny = 50.0; 
  Kd.l_leg_uay = 57.0;
  Kd.l_leg_lax = 20.0;
  Kd.neck_ay = 2.5; 
  Kd.back_lbz = 60.0; 
  Kd.back_mby = 40.0; 
  Kd.back_ubx = 23.0; 
  
else
  error('unknown mode given. valid modes are: default, pinned, stance_leg, gazebo.');
end

% copy left gains to right side
Kp.r_arm_usy = Kp.l_arm_usy;
Kp.r_arm_shx = Kp.l_arm_shx;
Kp.r_arm_ely = Kp.l_arm_ely;
Kp.r_arm_elx = Kp.l_arm_elx;
Kp.r_arm_uwy = Kp.l_arm_uwy;
Kp.r_arm_mwx = Kp.l_arm_mwx;
Kp.r_leg_uhz = Kp.l_leg_uhz;
Kp.r_leg_mhx = Kp.l_leg_mhx;
Kp.r_leg_lhy = Kp.l_leg_lhy;
Kp.r_leg_kny = Kp.l_leg_kny;
Kp.r_leg_uay = Kp.l_leg_uay;
Kp.r_leg_lax = Kp.l_leg_lax;

Kd.r_arm_usy = Kd.l_arm_usy;
Kd.r_arm_shx = Kd.l_arm_shx;
Kd.r_arm_ely = Kd.l_arm_ely;
Kd.r_arm_elx = Kd.l_arm_elx;
Kd.r_arm_uwy = Kd.l_arm_uwy;
Kd.r_arm_mwx = Kd.l_arm_mwx;
Kd.r_leg_uhz = Kd.l_leg_uhz;
Kd.r_leg_mhx = Kd.l_leg_mhx;
Kd.r_leg_lhy = Kd.l_leg_lhy;
Kd.r_leg_kny = Kd.l_leg_kny;
Kd.r_leg_uay = Kd.l_leg_uay;
Kd.r_leg_lax = Kd.l_leg_lax;

Kp = diag(double(Kp));
Kd = diag(double(Kd));

end