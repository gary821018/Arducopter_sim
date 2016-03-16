%{
x(1)


%}
function diff=test2(t2,y);

global mx my mm fx1 fx2 fy1 fy2 g 

	diff=zeros(12,1);
	diff(1)=y(2);
	diff(2)=(1/(mx+my+mm))*(fx1+fx2+fy1+fy2)*sin(y(9))+0.09*randn();
	diff(3)=y(4);
	diff(4)=-(1/(mx+my+mm))*(fx1+fx2+fy1+fy2)*sin(y(7))*cos(y(9))+0.09*randn();
	diff(5)=y(6);
	diff(6)=(1/(mx+my+mm))*(fx1+fx2+fy1+fy2)*cos(y(7))*cos(y(9))-g+0.13*randn();
	diff(7)=0;
	diff(8)=0;
	diff(9)=0;
	diff(10)=0;
	diff(11)=0;
	diff(12)=0;
end
