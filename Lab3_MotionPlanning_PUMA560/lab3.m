close;
clear;
clc;
%q for robot trajectory 
q=[transpose(linspace(0,pi,200)),transpose(linspace(0,pi/2,200)),transpose(linspace(0,pi,200)),transpose(linspace(pi/4,3*pi/4,200)),transpose(linspace(-pi/3,pi/3,200)),transpose(linspace(0,2*pi,200))];
%DH params 
DH=[0,76,0,1.57079632679490;0,-23.6500000000000,43.2300000000000,0;0,0,0,1.57079632679490;0,43.1800000000000,0,-1.57079632679490;0,0,0,1.57079632679490;0,20,0,0];
%create robot 
myrobot=mypuma560(DH);

%3.1
%testing att
H1 = eul2tr([0 pi pi/2]); 
H1(1:3,4) = 100*[-1; 3; 3;]/4;
q1 = inverse(H1, myrobot);
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4) = 100*[3;-1;2;]/4;
q2 = inverse(H2, myrobot);
tau = att(q1,q2,myrobot)

%3.2
%testing motion plan without obstacle
qref=motionplan(q1,q2,0,10,myrobot,[],0.005)
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);

%3.3
%testing rep 
setupobstacle;
q3 = 0.9*q1 + 0.1*q2;
tau = rep(q3,myrobot,obs{1})
q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot,obs{6})

%testing motion plan with obstacles
setupobstacle
hold on
axis([-100 100 -100 100 0 200]);
view(-32,50)
plotobstacle(obs);
qref=motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);
hold off;
