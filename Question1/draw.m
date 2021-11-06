clear
clc
N=2408;
load PIDy_data.txt
load PIDtheta_data.txt
load PIDu_data.txt
load PIDx_data.txt
x=PIDx_data(:,2);
y=PIDy_data(:,2);
theta=PIDtheta_data(:,2);
t=PIDu_data(:,1)/10;
u=PIDu_data(:,2);

figure(1);
hold on;
plot(x(1:N),y(1:N),'k','LineWidth',3);
grid on;
xlabel('x/m');
ylabel('y/m');
title('Route of the car');
q1=quiver(x(1),y(1),cos(theta(1)/180*pi),sin(theta(1)/180*pi),5,'LineWidth',2);
q2=quiver(x(N),y(N),cos(theta(N)/180*pi),sin(theta(N)/180*pi),5,'LineWidth',2);
q1.Marker='o';
q2.Marker='x';

hold off;


figure(2);
plot(t(1:N),theta(1:N),'k','LineWidth',3);
grid on;
xlabel('t/s');
ylabel('\theta/\circ');
title('Change of \theta with time');

figure(3);
plot(t(1:N),u(1:N),'k','LineWidth',3);
grid on;
xlabel('t/s');
ylabel('u/\circ');
title('Change of u with time');

figure(4);
plot(t(1:N),y(1:N),'k','LineWidth',3);
grid on;
xlabel('t/s');
ylabel('y/m');
title('Change of y with time');
