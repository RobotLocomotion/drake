[xtraj1,utraj1,ltraj1,v1,p1]=runDirtran;
%v1.playback(xtraj1);
xtraj_init = xtraj1;
utraj_init = utraj1;
ltraj_init = ltraj1;
%keyboard
[xtraj2,utraj2,ltraj2,v2,p2]=runDirtran(xtraj_init, utraj_init, ltraj_init,.1,.1);
%v2.playback(xtraj2);
xtraj_init = xtraj2;
utraj_init = utraj2;
ltraj_init = ltraj2;
%keyboard
[xtraj3,utraj3,ltraj3,v3,p3]=runDirtran(xtraj_init, utraj_init, ltraj_init,.01,.01);
%v3.playback(xtraj3);
xtraj_init = xtraj3;
utraj_init = utraj3;
ltraj_init = ltraj3;
%keyboard
[xtraj4,utraj4,ltraj4,v4,p4]=runDirtran(xtraj_init, utraj_init, ltraj_init,1e-3,1e-3);
%v4.playback(xtraj4);
xtraj_init = xtraj4;
utraj_init = utraj4;
ltraj_init = ltraj4;

[xtraj5,utraj5,ltraj5,v5,p5]=runDirtran(xtraj_init, utraj_init, ltraj_init,0,0);
v5.playback(xtraj5,struct('slider',true));
