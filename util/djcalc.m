function  dXj = djcalc( pitch, q )

if pitch == 0				% revolute joint
  dXj = dXrotz(q);
elseif pitch == inf			% prismatic joint
  dXj = dXtrans([0 0 q]); dXj = reshape(dXj(:,3),6,6);
else					% helical joint
  dXj = Xrotz(q) * reshape(dXtrans([0 0 q*pitch])*[0;0;pitch],6,6) + dXrotz(q) * Xtrans([0 0 q*pitch]);
end