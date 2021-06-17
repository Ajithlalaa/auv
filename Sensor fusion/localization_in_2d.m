%initializations
clear pos_x pos_y ori_si vel_u vel_v 

%%for the visualization of an object moving in 2D plane
close(figure(1))
ax = axes('XLim',[-50 50],'YLim',[-50 50]);
view(2)
grid on;
[x,y] = cylinder([.2 0.2]);
h(1) = surface(y,x,'FaceColor','yellow');
h(2) = surface(y,x,'FaceColor','blue');
grid on
xlabel('X');
ylabel('Y');
t = hgtransform('Parent',ax);
set(h,'Parent',t)
drawnow

%%

k=1000;
clc
M=20; %mass of 26 Kg
g=9.8; %9.8 m/s2
W=M*g; %Weight of the body
B=196.2; %Bouyancy force

A=0.045; %frontal area
C=0.5; %Drag coefficient
Ro=997; %density of the water

dt=0.1; %time step

lx1=0.12; %length between thruster1 mounted point and centre of mass.
lx2=0.12;
yp=lx1+lx2;

%moments of inertia
Izz=10; %not a calculated value.You need to calculate it :)

%Euler angles
si=0.01;

%input thrust
 Tx2=15;
 Tx1=15;

%angular velocities in body frame
r=0;

%velocities in body frame
u=0;
v=0;

%velocities in inertial frame
x_dot=0;
y_dot=0;

%pose in inertial frame
x=0;
y=0;

ti=0;

 

for i=1:k
          
%drag forces
%Dx=0.5*C*Ro*A*(u^2);
Dy=0.5*C*Ro*A*(v^2);

%derivative of velocities in body frame from equation of motion
u_dot=(r*v);
v_dot=(((Tx1+Tx2)-Dy)/M)-(r*u);

%derivatives of angular velocities in body frame
r_dot=((Tx1-Tx2)*yp)/Izz;

%velocities in body frame
u=u+(u_dot*dt);
v=v+(v_dot*dt);

wo=[u;v];

%angular velocities
r=r+(r_dot*dt);
ao=r;

%rotation matrix
R=[cos(si) -sin(si);
    sin(si) cos(si)];

%velocites in inertial frame
X_dot=R*wo;
x_dot=X_dot(1);
y_dot=X_dot(2);

%for derivative of angular velocities in inertial frame
si_dot=r;

%euler angles
si=si+(si_dot*dt);

%pose in inertial frame
x=x+(x_dot*dt);
y=y+(y_dot*dt);

pos_x(i)=x;
pos_y(i)=y;

vel_u(i)=u;
vel_v(i)=v;

ori_si(i)=si;

dragF(i)=Dy;

%for plot in 2D plane
translation=makehgtform('translate',[pos_x(i),pos_y(i),0]);
zrotational=makehgtform('zrotate',ori_si(i));
set(t,'matrix',translation*zrotational); 
pause(0.1)    

end

pos_x=pos_x';
pos_y=pos_y';
ori_si=ori_si';
vel_u=vel_u';
vel_v=vel_v';
dragF=dragF';
time=0:0.1:9.9;
figure(2)
plot(time,vel_v)
grid on
xlabel('Time');
ylabel('Velocity_v');
figure(3)
plot(time,dragF)
xlabel('Time');
ylabel('Force');



























