
function diffxyzphithetapsi=test(t,x);

global mx my mm fx1 fx2 fy1 fy2 tx1 tx2 ty1 ty2 Ixx Iyy Izz lx1 lx2 ly1 ly2 g

	diffxyzphithetapsi=zeros(12,1);
	diffxyzphithetapsi(1)=x(2);
	diffxyzphithetapsi(2)=(1/(mx+my+mm))*(fx1+fx2+fy1+fy2)*sin(x(9));
	diffxyzphithetapsi(3)=x(4);
	diffxyzphithetapsi(4)=-(1/(mx+my+mm))*(fx1+fx2+fy1+fy2)*sin(x(7))*cos(x(9)) ;
	diffxyzphithetapsi(5)=x(6);
	diffxyzphithetapsi(6)=(1/(mx+my+mm))*(fx1+fx2+fy1+fy2)*cos(x(7))*cos(x(9))-g ;
	diffxyzphithetapsi(7)=(cos(x(11))/cos(x(9)))*x(8)-(sin(x(11))/cos(x(9)))*x(10);
	diffxyzphithetapsi(8)=(1/Ixx)*(-fy1*ly1+fy2*ly2+(Iyy-Izz)*x(10)*x(12));
	diffxyzphithetapsi(9)=sin(x(11))*x(8)+cos(x(11))*x(10);
	diffxyzphithetapsi(10)=(1/Iyy)*(fx1*lx1-fx2*lx2+(-Ixx+Izz)*x(8)*x(12));
	diffxyzphithetapsi(11)=-(sin(x(9))*cos(x(11))/cos(x(9)))*x(8)+(sin(x(9))*sin(x(11))/cos(x(9)))*x(10)+x(12);
	diffxyzphithetapsi(12)=(1/Izz)*((Ixx-Iyy)*x(8)*x(10)+(tx1+tx2-ty1-ty2));
end
