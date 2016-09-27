function ddvcrossw = ddcross(v,w,dv,dw,ddv,ddw)
dims = 3;
ddvcrossw = zeros(3*dims,dims);

ddvcrossw(1,1) = ddv(2,1)*w(3) + dv(2,1)*dw(3,1) + dv(2,1)*dw(3,1) + v(2)*ddw(3,1) - ddv(3,1)*w(2) - dv(3,1)*dw(2,1) - dv(3,1)*dw(2,1) - v(3)*ddw(2,1);
ddvcrossw(2,1) = ddv(3,1)*w(1) + dv(3,1)*dw(1,1) + dv(3,1)*dw(1,1) + v(3)*ddw(1,1) - ddv(1,1)*w(3) - dv(1,1)*dw(3,1) - dv(1,1)*dw(3,1) - v(1)*ddw(3,1);
ddvcrossw(3,1) = ddv(1,1)*w(2) + dv(1,1)*dw(2,1) + dv(1,1)*dw(2,1) + v(1)*ddw(2,1) - ddv(2,1)*w(1) - dv(2,1)*dw(1,1) - dv(2,1)*dw(1,1) - v(2)*ddw(1,1);
ddvcrossw(4,1) = ddv(5,1)*w(3) + dv(2,2)*dw(3,1) + dv(2,1)*dw(3,2) + v(2)*ddw(6,1) - ddv(6,1)*w(2) - dv(3,2)*dw(2,1) - dv(3,1)*dw(2,2) - v(3)*ddw(5,1);
ddvcrossw(5,1) = ddv(6,1)*w(1) + dv(3,2)*dw(1,1) + dv(3,1)*dw(1,2) + v(3)*ddw(4,1) - ddv(4,1)*w(3) - dv(1,2)*dw(3,1) - dv(1,1)*dw(3,2) - v(1)*ddw(6,1);
ddvcrossw(6,1) = ddv(4,1)*w(1) + dv(1,2)*dw(2,1) + dv(1,1)*dw(2,2) + v(1)*ddw(5,1) - ddv(5,1)*w(1) - dv(2,2)*dw(1,1) - dv(2,1)*dw(1,2) - v(2)*ddw(4,1);
ddvcrossw(7,1) = ddv(8,1)*w(3) + dv(2,3)*dw(3,1) + dv(2,1)*dw(3,3) + v(2)*ddw(9,1) - ddv(9,1)*w(2) - dv(3,3)*dw(2,1) - dv(3,1)*dw(2,3) - v(3)*ddw(8,1);
ddvcrossw(8,1) = ddv(9,1)*w(1) + dv(3,3)*dw(1,1) + dv(3,1)*dw(1,3) + v(3)*ddw(7,1) - ddv(7,1)*w(3) - dv(1,3)*dw(3,1) - dv(1,1)*dw(3,3) - v(1)*ddw(9,1);
ddvcrossw(9,1) = ddv(7,1)*w(2) + dv(1,3)*dw(2,1) + dv(1,1)*dw(2,3) + v(1)*ddw(8,1) - ddv(8,1)*w(1) - dv(2,3)*dw(1,1) - dv(2,1)*dw(1,3) - v(2)*ddw(7,1);

ddvcrossw(1,2) = ddv(2,2)*w(3) + dv(2,1)*dw(3,2) + dv(2,2)*dw(3,1) + v(2)*ddw(3,2) - ddv(3,2)*w(2) - dv(3,1)*dw(2,2) - dv(3,2)*dw(2,1) - v(3)*ddw(2,2);
ddvcrossw(2,2) = ddv(3,2)*w(1) + dv(3,1)*dw(1,2) + dv(3,2)*dw(1,1) + v(3)*ddw(1,2) - ddv(1,2)*w(3) - dv(1,1)*dw(3,2) - dv(1,2)*dw(3,1) - v(1)*ddw(3,2);
ddvcrossw(3,2) = ddv(1,2)*w(2) + dv(1,1)*dw(2,2) + dv(1,2)*dw(2,1) + v(1)*ddw(2,2) - ddv(2,2)*w(1) - dv(2,1)*dw(1,2) - dv(2,2)*dw(1,1) - v(2)*ddw(1,2);
ddvcrossw(4,2) = ddv(5,2)*w(3) + dv(2,2)*dw(3,2) + dv(2,2)*dw(3,2) + v(2)*ddw(6,2) - ddv(6,2)*w(2) - dv(3,2)*dw(2,2) - dv(3,2)*dw(2,2) - v(3)*ddw(5,2);
ddvcrossw(5,2) = ddv(6,2)*w(1) + dv(3,2)*dw(1,2) + dv(3,2)*dw(1,2) + v(3)*ddw(4,2) - ddv(4,2)*w(3) - dv(1,2)*dw(3,2) - dv(1,2)*dw(3,2) - v(1)*ddw(6,2);
ddvcrossw(6,2) = ddv(4,2)*w(2) + dv(1,2)*dw(2,2) + dv(1,2)*dw(2,2) + v(1)*ddw(5,2) - ddv(5,2)*w(1) - dv(2,2)*dw(1,2) - dv(2,2)*dw(1,2) - v(2)*ddw(4,2);
ddvcrossw(7,2) = ddv(8,2)*w(3) + dv(2,3)*dw(3,2) + dv(2,2)*dw(3,3) + v(2)*ddw(9,2) - ddv(9,2)*w(2) - dv(3,3)*dw(2,2) - dv(3,2)*dw(2,3) - v(3)*ddw(8,2);
ddvcrossw(8,2) = ddv(9,2)*w(1) + dv(3,3)*dw(1,2) + dv(3,2)*dw(1,3) + v(3)*ddw(7,2) - ddv(7,2)*w(3) - dv(1,3)*dw(3,2) - dv(1,2)*dw(3,3) - v(1)*ddw(9,2);
ddvcrossw(9,2) = ddv(7,2)*w(2) + dv(1,3)*dv(2,2) + dv(1,2)*dw(2,3) + v(1)*ddw(8,2) - ddv(8,2)*w(1) - dv(2,3)*dw(1,2) - dv(2,2)*dw(1,3) - v(2)*ddw(7,2);

ddvcrossw(1,3) = ddv(2,3)*w(3) + dv(2,1)*dw(3,3) + dv(2,3)*dw(3,1) + v(2)*ddw(3,3) - ddv(3,3)*w(2) - dv(3,1)*dw(2,3) - dv(3,3)*dw(2,1) - v(3)*ddw(2,3);
ddvcrossw(2,3) = ddv(3,3)*w(1) + dv(3,1)*dw(1,3) + dv(3,3)*dw(1,1) + v(3)*ddw(1,3) - ddv(1,3)*w(3) - dv(1,1)*dw(3,3) - dv(1,3)*dw(3,1) - v(1)*ddw(3,3);
ddvcrossw(3,3) = ddv(1,3)*w(2) + dv(1,1)*dw(2,3) + dv(1,3)*dw(2,1) + v(1)*ddw(2,3) - ddv(2,3)*w(1) - dv(2,1)*dw(1,3) - dv(2,3)*dw(1,1) - v(2)*ddw(1,3);
ddvcrossw(4,3) = ddv(5,3)*w(3) + dv(2,2)*dw(3,3) + dv(2,3)*dw(3,2) + v(2)*ddw(6,3) - ddv(6,3)*w(2) - dv(3,2)*dw(2,3) - dv(3,3)*dw(2,2) - v(3)*ddw(5,3);
ddvcrossw(5,3) = ddv(6,3)*w(1) + dv(3,2)*dw(1,3) + dv(3,3)*dw(1,2) + v(3)*ddw(4,3) - ddv(4,3)*w(3) - dv(1,2)*dw(3,3) - dv(1,3)*dw(3,2) - v(1)*ddw(6,3);
ddvcrossw(6,3) = ddv(4,3)*w(2) + dv(1,2)*dw(2,3) + dv(1,3)*dw(2,2) + v(1)*ddw(5,3) - ddv(5,3)*w(1) - dv(2,2)*dw(1,3) - dv(2,3)*dw(1,2) - v(2)*ddw(4,3);
ddvcrossw(7,3) = ddv(8,3)*w(3) + dv(2,3)*dw(3,3) + dv(2,3)*dw(3,3) + v(2)*ddw(9,3) - ddv(9,3)*w(2) - dv(3,3)*dw(2,3) - dv(3,3)*dw(2,3) - v(3)*ddw(8,3);
ddvcrossw(8,3) = ddv(9,3)*w(1) + dv(3,3)*dw(1,3) + dv(3,3)*dw(1,3) + v(3)*ddw(7,3) - ddv(7,3)*w(3) - dv(1,3)*dw(3,3) - dv(1,3)*dw(3,3) - v(1)*ddw(9,3);
ddvcrossw(9,3) = ddv(7,3)*w(2) + dv(1,3)*dw(2,3) + dv(1,3)*dw(2,3) + v(1)*ddw(8,3) - ddv(8,3)*w(1) - dv(2,3)*dw(1,3) - dv(2,3)*dw(1,3) - v(2)*ddw(7,3);

end