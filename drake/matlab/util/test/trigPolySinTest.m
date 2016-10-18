function trigPolySinTest

checkDependency('spotless');
q_mss=msspoly('q',2);
s_mss=msspoly('s',2);
c_mss=msspoly('c',2);
q_trig=TrigPoly(q_mss,s_mss,c_mss);

f1 = sin(q_trig(1)*2 + 1);
f2 = sin(1)*(c_mss(1)^2 - s_mss(1)^2) + cos(1)*2*s_mss(1)*c_mss(1);

assert(isequal(f1,f2))

end