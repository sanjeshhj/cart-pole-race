function mainCartPoleRace
% Cart-Pole Race
% Start from x=0, theta=pi; End at x=20
% theta not to deviate more than +/-15 from pi all throughout

clear all
close all
clc

% step size and maximum time steps
timeStep = 0.01;
maxSteps = 2000;

% initial state
s = [0 0 -pi 0];

% initialize a matrix to store state for plotting
S = zeros(maxSteps,size(s,2));

% initialize figure
figure('units','normalized','outerposition',[0 0 1 1])

% iterate till cart reaches finishing point or a maximum time limit
i = 0;
while s(1)<20&&i<maxSteps
        
    % state update RK4 variable time step
    [~,s_sol] = ode45(@dynamicsCartPoleRK4,[i*timeStep (i+1)*timeStep],s);
    s = s_sol(end,:);
   
    % animation	
    plotCartPole(s);    
    
    % store state for plotting
    i = i+1;
    S(i,:) = s;
   
end

% post-processing after the race
T = timeStep*(1:i);
S = S(1:length(T),:);

% plot the result on animation figure
if s(1)<20
    disp('Race not completed!')
    text(10,1,'Race not completed!','Color',[1 1 1],'BackgroundColor',[1 .3 .3],'Fontsize',24,'interpreter','latex','HorizontalAlignment','Center')
else
    if and((S(:,3)*180/pi<-165),(S(:,3)*180/pi>-195))
        disp(['Cart took ' num2str(T(end)) ' seconds to complete the race!'])
        text(10,2,[num2str(T(end)) ' seconds!'],'Color',[1 1 1],'BackgroundColor',[.3 1 .3],'Fontsize',24,'HorizontalAlignment','Center')
    else
        disp('INVALID RACE! Pendulum angle bounds were not met!!')
        disp(['Cart took ' num2str(T(end)) ' seconds to complete the race'])
        text(10,2,[num2str(T(end)) ' seconds!'],'Color',[1 1 1],'BackgroundColor',[1 .3 .3],'Fontsize',24,'HorizontalAlignment','Center')
        text(10,1,'$\theta$ bounds not met','Color',[1 1 1],'BackgroundColor',[1 .3 .3],'Fontsize',24,'interpreter','latex','HorizontalAlignment','Center')
    end
end

% plot cart position with time
figure('units','normalized','outerposition',[0 0 1 1])
plot(T,S(:,1))
xlabel('$t$','interpreter','latex','FontSize',18); 
ylabel('$x$ (m)','interpreter','latex','FontSize',18)

% plot pendulum angle with time (with bounds)
figure('units','normalized','outerposition',[0 0 1 1])
plot(T,S(:,3)*180/pi,'b')
xlabel('$t$','interpreter','latex','FontSize',18); 
ylabel('$\theta$ (deg)','interpreter','latex','FontSize',18)
hold on
plot(T,165,'r--',T,195,'r--')

figure(1)
% End of main function


function [ds] = dynamicsCartPoleRK4(t,s)
% state s; control u; RK4 update

% states
x_dot      = s(2);
theta      = s(3);
theta_dot  = s(4);

% parameters
g    = 9.81;     % acceleration due to gravity
M    = 1.0;      % mass of the cart
m    = 0.1;      % mass of the pole
l    = 1.0;      % length of the pole 
Fmax = 10.0;

% control law
F = mycontrol(s);

% bound the actuation force between -Fmax and Fmax
%F = max(min(F,Fmax),-Fmax);

% compute acceleration
%a = ([cos(theta) l; (M+m) m*l*cos(theta)])\[-g*sin(theta);(F+m*l*theta_dot^2*sin(theta))];
A=[0 1 0 0;
   0 0 m*g/M 0;
   0 0 0 1;
   0 0 (M+m)*g/(M*l) 0];

B=[0;1/M;0;1/(M*l)];
ds=A*[s(1);x_dot;theta;theta_dot]+B*F;
% derivatives
%ds = [x_dot a(1) theta_dot a(2)]';

% End of function


function plotCartPole(s)
% function to plot cart pole in animation

% parameters
x = s(1);
theta = pi-s(3);
l=3;
h=1.0;
pxg = [x+1 x-1 x-1 x+1 x+1];
pyg = [0.25 0.25 h h 0.25];

pxp=[x x+l*sin(theta)];
pyp=[h h+l*cos(theta)];

%Base
plot([-25 25],[0 0],'color',[.5 .5 .5])
hold on

%Start Line
plot([0 0],[-2 4],'Color',[.5 1 .5],'LineWidth',4)

%Finish Line
plot([20 20],[-2 4],'Color',[1 .5 .5],'LineWidth',4)

%Car 
fill(pxg,pyg,[.6 .6 .5],'LineWidth',1);

%Car Wheels
plot(x-0.5,0.25,'rO','LineWidth',1,'Markersize',20,'MarkerEdgeColor','k','MarkerFaceColor',[0.5 0.5 0.5]);
plot(x+0.5,0.25,'rO','LineWidth',1,'Markersize',20,'MarkerEdgeColor','k','MarkerFaceColor',[0.5 0.5 0.5]);

%Pendulum
plot(pxp,pyp,'-k','LineWidth',5);
plot(pxp(1),pyp(1),'o','LineWidth',1,'Markersize',8,'MarkerEdgeColor','k','MarkerFaceColor','k');
plot(pxp(2),pyp(2),'rO','LineWidth',1,'Markersize',20,'MarkerEdgeColor','k','MarkerFaceColor',[.6 .6 .6]);

axis equal
axis([-25 25 -2 4])

set(gca,'yTick',[],'yTickLabel',[],'yColor',[1 1 1])

box off

drawnow;
hold off