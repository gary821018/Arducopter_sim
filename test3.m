
function diffxyzphithetapsi=test3(t3,z);

global mx my mm fx1 fx2 fy1 fy2 tx1 tx2 ty1 ty2 Ixx Iyy Izz lx1 lx2 ly1 ly2 g

	diffxyzphithetapsi=zeros(12,1);
	diffxyzphithetapsi(1)=0;
	diffxyzphithetapsi(2)=0;
	diffxyzphithetapsi(3)=0;
	diffxyzphithetapsi(4)=0;
	diffxyzphithetapsi(5)=0;
	diffxyzphithetapsi(6)=0;
	diffxyzphithetapsi(7)=(cos(z(11))/cos(z(9)))*z(8)-(sin(z(11))/cos(z(9)))*z(10);
	diffxyzphithetapsi(8)=(1/Ixx)*(-fy1*ly1+fy2*ly2+(Iyy-Izz)*z(10)*z(12));
	diffxyzphithetapsi(9)=sin(z(11))*z(8)+cos(z(11))*z(10);
	diffxyzphithetapsi(10)=(1/Iyy)*(fx1*lx1-fx2*lx2+(-Ixx+Izz)*z(8)*z(12));
	diffxyzphithetapsi(11)=-(sin(z(9))*cos(z(11))/cos(z(9)))*z(8)+(sin(z(9))*sin(z(11))/cos(z(9)))*z(10)+z(12);
	diffxyzphithetapsi(12)=(1/Izz)*((Ixx-Iyy)*z(8)*z(10)+(tx1+tx2-ty1-ty2));
end
