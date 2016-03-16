clear all
clc

global mx my mm fx1 fx2 fy1 fy2 tx1 tx2 ty1 ty2 Ixx Iyy Izz lx1 lx2 ly1 ly2 g z_d kp_z kd_z x_d kp_x kd_x kp_theta kd_theta g_theta y_d kp_y kd_y kp_phi kd_phi g_phi test_ax test_ay test_az test_wphi test_wtheta test_wpsi v_x v_y v_z   
% 目標位置 kp kd
z_d=20;
kp_z=1;
kd_z=2;

x_d=5;
kp_x=0.55;
kd_x=1.5;

y_d=10;
kp_y=0.5;
kd_y=1.4;

kp_theta=1.5;
kd_theta=1;

kp_phi=-1.5;
kd_phi=-1;
%------------------------------------------------

% 初始值
X0=0;	% initial X
Y0=0;	% initial Y
Z0=0;	% initial Z
Vx0=0;	% initial Vx
Vy0=0;	% initial Vy
Vz0=0;	% initial Vz
phi0=0;
w_phi0=0;
theta0=0;
w_theta0=0;
psi0=0;
w_psi0=0;

C_T=0.14;
C_Q=0.05;
rho_air=1.1755;
A=0.0049898;
R=0.1225;

w_co_f=C_T*rho_air*A*R^3;
w_co_t=C_Q*rho_air*A*R^3;

%w_rotor_x1=2500;%1811.565;
%w_rotor_x2=1950;%1811.565;
%w_rotor_y1=1950;%1811.565;
%w_rotor_y2=1950;%1811.565;

%fx1=w_co_f*(w_rotor_x1)^2;
%fx2=w_co_f*(w_rotor_x2)^2;
%fy1=w_co_f*(w_rotor_y1)^2;
%fy2=w_co_f*(w_rotor_y2)^2;

%tx1=w_co_t*(w_rotor_x1)^2;
%tx2=w_co_t*(w_rotor_x2)^2;
%ty1=w_co_t*(w_rotor_y1)^2;
%ty2=w_co_t*(w_rotor_y2)^2;



mx1=0.093;
mx2=0.093;
my1=0.093;
my2=0.093;
mlx=0.105;
mly=0.105;
mm=1.08;

mx=mly+mx1+mx2;
my=mly+my1+my2;

g=9.8;

lx1=0.415;
lx2=0.415;
ly1=0.415;
ly2=0.415;

rhox=mlx/(lx1+lx2);
rhoy=mly/(ly1+ly2);

Ixx=(1/3)*rhoy*(ly2^3+ly1^3)+my1*ly1^2+my2*ly2^2;
Iyy=(1/3)*rhox*(lx2^3+lx1^3)+mx1*lx1^2+mx2*lx2^2;
Izz=Ixx+Iyy;
%------------------------------------------------------

% PD controller 位置控制

% X axis
Fcx=kp_x*(x_d);


% Z axis
Fcz=(mx+my+mm)*g;

% Y axis
Fcy=kp_y*(y_d);

%------------------------------------------------------

% PD controller 角度控制
F=Fcz/(cos(theta0)*cos(phi0));
g_theta=asin(Fcx/F);

g_phi=asin(Fcy/((-1)*F*cos(theta0)));
%
tc_theta=kp_theta*(g_theta-theta0)+kd_theta*(0-w_theta0);
tc_phi=kp_phi*(g_phi-phi0)+kd_phi*(0-w_phi0);
%------------------------------------------------------

% 馬達升力 力矩

w_rotor_x1=(1/2)*((F/(2*w_co_f))+(tc_theta/(lx1*w_co_f)));
w_rotor_x2=(1/2)*((F/(2*w_co_f))-(tc_theta/(lx1*w_co_f)));
w_rotor_y1=(1/2)*((F/(2*w_co_f))+(tc_phi/(lx1*w_co_f)));
w_rotor_y2=(1/2)*((F/(2*w_co_f))-(tc_phi/(lx1*w_co_f)));

fx1=w_co_f*(w_rotor_x1);
fx2=w_co_f*(w_rotor_x2);
fy1=w_co_f*(w_rotor_y1);
fy2=w_co_f*(w_rotor_y2);

tx1=w_co_t*(w_rotor_x1);
tx2=w_co_t*(w_rotor_x2);
ty1=w_co_t*(w_rotor_y1);
ty2=w_co_t*(w_rotor_y2);
%-----------------------------------------------------------

k=1;
y=zeros(12,12);
count=1;
count2=1;
num=0;

sensor_x=0;
sensor_y=0;
sensor_z=0;

v_x=0;
v_y=0;
v_z=0;

theta=0;
phi=0;

w_phi=0;
w_theta=0;

% 四旋翼姿態解算
for j = 1:300000
[t,x]=ode45(@test,[0,0.0001],[X0,Vx0,Y0,Vy0,Z0,Vz0,phi0,w_phi0,theta0,w_theta0,psi0,w_psi0]);

%-------------------------------------------------------------------------------
i=length(x);

X0=x(i,1);
Vx0=x(i,2);
Y0=x(i,3);
Vy0=x(i,4);
Z0=x(i,5);
Vz0=x(i,6);
phi0=x(i,7);
w_phi0=x(i,8);
theta0=x(i,9);
w_theta0=x(i,10);
psi0=x(i,11);
w_psi0=x(i,12);



%---------------------------------------------------------------------------------

% GPS

if count2 == 1000

% X axis

sensor_x=x(i,1)+2*randn();

%
% Z axis

sensor_z=x(i,5)+2*randn();


% Y axis

sensor_y=x(i,3)+2*randn();

%

count2=1;
else
    count2=count2+1;
end;

% IMU
if count == 10
    test_wphi=w_phi0+0.017*randn();
    test_wtheta=w_theta0+0.017*randn();
    test_wpsi=w_psi0+0.017*randn();
    [t2,y]=ode45(@test2,[0,0.001],[sensor_x,v_x,sensor_y,v_y,sensor_z,v_z,phi0,w_phi0,theta0,w_theta0,psi0,w_psi0]);
    [t3,z]=ode45(@test3,[0,0.001],[X0,Vx0,Y0,Vy0,Z0,Vz0,phi,test_wphi,theta,test_wtheta,psi,test_wpsi]);
    
    
    
    k=length(y);
    m=length(z);
    
    
    count = 1;
    
    sensor_x=y(k,1);
    sensor_y=y(k,3);
    sensor_z=y(k,5);
    
    
% IMU積分速度誤差
    v_x=y(k,2);
    v_y=y(k,4);
    v_z=y(k,6);
% IMU角速度誤差
    w_phi=test_wphi;
    w_theta=test_wtheta;
% IMU角度誤差
    phi=z(m,7);
    theta=z(m,9);
    psi=z(m,11);
else 
    count=count+1;
end;  
%--------------------------------------------------------------------







%----------------------------------------------------------------------


% PD控制
Fcx=kp_x*(x_d-sensor_x)+kd_x*(0-v_x);
Fcy=kp_y*(y_d-sensor_y)+kd_y*(0-v_y);
Fcz=(mx+my+mm)*g+kp_z*(z_d-sensor_z)+kd_z*(0-v_z);

%theta=x(i,9);
%phi=x(i,7);
F=Fcz/(cos(theta)*cos(phi));

g_theta=asin(Fcx/F);
g_phi=asin(Fcy/((-1)*F*cos(theta)));
%w_theta=x(i,10);
%w_phi=x(i,8);
tc_theta=kp_theta*(g_theta-theta)+kd_theta*(0-w_theta);
tc_phi=kp_phi*(g_phi-phi)+kd_phi*(0-w_phi);
%-----------------------------------------------------------------------------


% 馬達計算
w_rotor_x1=sqrt((1/2)*((F/(2*w_co_f))+(tc_theta/(lx1*w_co_f))));
w_rotor_x2=sqrt((1/2)*((F/(2*w_co_f))-(tc_theta/(lx1*w_co_f))));
w_rotor_y1=sqrt((1/2)*((F/(2*w_co_f))+(tc_phi/(lx1*w_co_f))));
w_rotor_y2=sqrt((1/2)*((F/(2*w_co_f))-(tc_phi/(lx1*w_co_f))));

fx1=w_co_f*(w_rotor_x1)^2;
fx2=w_co_f*(w_rotor_x2)^2;
fy1=w_co_f*(w_rotor_y1)^2;
fy2=w_co_f*(w_rotor_y2)^2;

tx1=w_co_t*(w_rotor_x1)^2;
tx2=w_co_t*(w_rotor_x2)^2;
ty1=w_co_t*(w_rotor_y1)^2;
ty2=w_co_t*(w_rotor_y2)^2;
%------------------------------------------------------------------------------

   



psi=x(i,11);

% 圖上之點
draw_x(j)=x(i,1);
draw_y(j)=x(i,3);
draw_z(j)=x(i,5);

draw_phi(j)=x(i,7);
draw_theta(j)=x(i,9);
draw_psi(j)=x(i,11);

draw_vx(j)=x(i,2);
draw_vy(j)=x(i,4);
draw_vz(j)=x(i,6);
%-----------------------------------------------------------------
num=num+1;



end;
%--------------------------------------------------------------

% 畫圖
draw_t=0.0001:0.0001:j/10000;
plot(draw_t,draw_x,'r-');
xlabel('time');   
ylabel('x distance');              
grid on;
figure(2);

plot(draw_t,draw_y,'r-');
xlabel('time');   
ylabel('y distance');              
grid on;       
figure(3);

plot(draw_t,draw_z,'r-');
xlabel('time');   
ylabel('z distance');              
grid on;       
figure(4);

%------------------------------------------------------

plot(draw_t,draw_phi,'r-');
xlabel('time');   
ylabel('phi_x');              
grid on;       
figure(5);

plot(draw_t,draw_theta,'r-');
xlabel('time');   
ylabel('theta_y');              
grid on;       
figure(6);

plot(draw_t,draw_psi,'r-');
xlabel('time');   
ylabel('psi_z');              
grid on;       
figure(7);


%------------------------------------------------------------

plot(draw_t,draw_vx,'r-');
xlabel('time');   
ylabel('v_x');              
grid on;       
figure(8);

plot(draw_t,draw_vy,'r-');
xlabel('time');   
ylabel('v_y');              
grid on;
figure(9);


plot(draw_t,draw_vz,'r-');
xlabel('time');   
ylabel('v_z');              
grid on;       
figure(10);       

