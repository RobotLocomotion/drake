function ret = relativeTdot(TBase, TBody, TBaseDot, TBodyDot)
% TRelative = inv(TBase) * TBody
% d/dt(TRelative) = d/dt(inv(TBase) * TBody) + inv(TBase) * TBodyDot
% d/dt(inv(Tbase) = -inv(TBase) * TBaseDot * inv(Tbase)

invTBasedot = -TBase \ TBaseDot / TBase;
ret = invTBasedot * TBody + TBase \ TBodyDot;

end