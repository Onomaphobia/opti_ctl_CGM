clc
clear variables
close all

% Links length[cm]
L1 = 30;
L2 = 30;
L3 = 30;

% initial angles[degree]
a1 = 30;
a2 = 30;
a3 = 30;

a1 = deg2rad(a1);
a2 = deg2rad(a2);
a3 = deg2rad(a3);

% a1i=0;
% a2i=0;
% a3i=0; 

% Joint Limit[rad]
% qmax = [1.7017;
%         1.047;
%         3.0542;
%         2.618;
%         3.059;
%         2.094;
%         3.059];
% qmin = -[1.7017;
%         2.147;
%         3.0542;
%         0.05;
%         3.059;
%         1.5708;  
%         3.059];
% qmid = (qmin+qmax)./2;

disp('Please enter the goal position and orientation of the end-effector in xyz coordinates')
xg = input('enter the x coordinate:');
yg = input('enter the y coordinate:');

v  = 10;% input('Please enter the desired linear velocity of the gripper in cm/s:')

% initial position transformation matrices use forward kinmatics function
% the Forward function is to calculate the transformation matrix using the
% initial angles of revolute joints and initial length of prismatic joint

disp('The initial transformation matrix is ')
[Ti01,Ti02,Ti03,Ti04] = Forward(L1,L2,L3,a1,a2,a3);
Ti04
% Initial position vector of the end-effector
disp('the initial position of the end-effector')
xi = Ti04(1,4);
yi = Ti04(2,4);

% Calculating the linear distance from the initial position to the desired position and the linear velocity:
D=sqrt((xg-Ti04(1,4))^2 + (yg-Ti04(2,4))^2); %Distance from initial position to goal position
dt=0.05; % Time increment in seconds.
time=D/v;  % Total time of animation.
n=round(time/dt); % Number of iterations rounded up.
dt=time/n; % Adjusted time increment in seconds. 

% the constant commanded Cartesian rates
Xdot = (xg-Ti04(1,4))/n; %[cm/sec]
Ydot = (yg-Ti04(2,4))/n; %[cm/sec]
dx = [Xdot;Ydot];

% % "1" for a Polynomial function with Blending, or "2" for a Polynomial function without Blending, or "3" for a Linear function.
% Tt = traj(1, Ti07, Tg, n+1);

% % Plot the simulation figure with initial position
% figure('Name','Arm animation','NumberTitle','off')
% % Plots of the Arm
% hold on
% link0=plot3([0,Ti01(1,4)],[0,Ti01(2,4)],[0,Ti01(3,4)],'color',1-0.25*(1-[1,0,0]),'LineWidth',5);
% link1=plot3([Ti01(1,4), Ti02(1,4)],[Ti01(2,4), Ti02(2,4)],[Ti01(3,4), Ti02(3,4)],'color',1-0.25*(1-[0,1,0]),'LineWidth',5);
% link2=plot3([Ti02(1,4), Ti03(1,4)],[Ti02(2,4), Ti03(2,4)],[Ti02(3,4), Ti03(3,4)],'color',1-0.25*(1-[0,0,1]),'LineWidth',5);
% link3=plot3([Ti03(1,4), Ti04(1,4)],[Ti03(2,4), Ti04(2,4)],[Ti03(3,4), Ti04(3,4)],'color',1-0.25*(1-[1,0,0]),'LineWidth',5);
% 
% 
% joint1=plot3(Ti01(1,4),Ti01(2,4),Ti01(3,4),'-ko','MarkerEdgeColor',1-0.25*(1-[0,0,0]),'MarkerFaceColor',1-0.25*(1-[.49 1 .63]),'MarkerSize',10);
% joint2=plot3(Ti02(1,4),Ti02(2,4),Ti02(3,4),'-ko','MarkerEdgeColor',1-0.25*(1-[0,0,0]),'MarkerFaceColor',1-0.25*(1-[.49 1 .63]),'MarkerSize',10);
% joint3=plot3(Ti03(1,4),Ti03(2,4),Ti03(3,4),'-ko','MarkerEdgeColor',1-0.25*(1-[0,0,0]),'MarkerFaceColor',1-0.25*(1-[.49 1 .63]),'MarkerSize',10);
% endeffector=plot3(Ti04(1,4),Ti04(2,4),Ti04(3,4),'-o','color',1-0.25*(1-[0,0,0]),'LineWidth',5,'MarkerSize',5);
% 
% % x=-300:100:300; % simulation plane 
% % y=x;
% % [x,y]=meshgrid(x,y);
% % z=x*0;
% % surf(x,y,z);
% % % shading interp
% % alpha(0.6)
% grid on;
% axis([-120 120 -120 120 -120 120 -120 120])
% view(2);
% 
% % Plots of points of interest on the system:
% initial=plot3(Ti04(1,4),Ti04(2,4),Ti04(3,4),'-ro','LineWidth',5,'MarkerSize',5);
% 
% goal=plot3(xg,yg,0,'-go','LineWidth',5,'MarkerSize',5);
% title('ARM Animation'); xlabel('x, (cm)'); ylabel('y (cm)'); zlabel('z (cm)');
% hold off;

%pause

% Initialize the variables needed for plotting at t=0
% These are for the plots that will be displayed at the end, not for the
% simulation

%data initialization
t = zeros(n+1,1);
X = zeros(n+1,1);
y = zeros(n+1,1);
Theta = zeros(3,n+1);
lambda = zeros(3,n+1);
U = zeros(3,n);
G = zeros(3,n);
G_pre = zeros(3,n);
Scale = zeros(3,n);
Scale_pre = zeros(3,n);
DFDQT = zeros(3,3);
DLDQ = zeros(3,1);
Thetadot = zeros(3,n+1);
cost_pinv = zeros(n+1,1);
cost_pinvTP = zeros(n+1,1);
cost_CGM = zeros(n+1,1);
sum_cost = zeros(101,1);
alpha = 0.2;
beta = 1;

i = 1;
count = 1;
t(1) = 0;            % Time  [sec]
Theta(1,1) = a1;    % Theta_1  [deg]
Theta(2,1) = a2;    % Theta_2  [deg]
Theta(3,1) = a3;    % Theta_3  [deg]
X(1)  = xi;          % Cartesian componet X [cm]
Y(1)  = yi;          % Cartesian componet Y [cm]
cost_pinv(1,1) = cost(L1,L2,L3,Theta(1,1),Theta(2,1),Theta(3,1));
cost_pinvTP(1,1) = cost(L1,L2,L3,Theta(1,1),Theta(2,1),Theta(3,1));
cost_CGM(1,1) = cost(L1,L2,L3,Theta(1,1),Theta(2,1),Theta(3,1));
i = i + 1;

%first loop of CGM, U is zero


%test of IK with task priority
while i < n + 2
    %[pinvJ]= pinvJ_2D(L1,L2,L3,a1,a2,a3);
    %dq = pinvJ*dx;
    
    Thetadot(:,i) = QdotTP(L1,L2,L3,Theta(1,i-1),Theta(2,i-1),Theta(3,i-1),Xdot,Ydot);
    Theta(1,i) = Theta(1,i-1) + Thetadot(1,i);
    Theta(2,i) = Theta(2,i-1) + Thetadot(2,i);
    Theta(3,i) = Theta(3,i-1) + Thetadot(3,i);
    cost_pinvTP(i,1) = cost(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
%     [T01,T02,T03,T04] = Forward(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
    % Update the simulation plot using matlab function set
%     set(link1,'XData',[T01(1,4), T02(1,4)],'YData',[T01(2,4), T02(2,4)],'ZData',[T01(3,4), T02(3,4)]);
%     set(link2,'XData',[T02(1,4), T03(1,4)],'YData',[T02(2,4), T03(2,4)],'ZData',[T02(3,4), T03(3,4)]);
%     set(link3,'XData',[T03(1,4), T04(1,4)],'YData',[T03(2,4), T04(2,4)],'ZData',[T03(3,4), T04(3,4)]);
%     
%     set(joint1,'XData',T01(1,4),'YData',T01(2,4),'ZData',T01(3,4));    
%     set(joint2,'XData',T02(1,4),'YData',T02(2,4),'ZData',T02(3,4));
%     set(joint3,'XData',T03(1,4),'YData',T03(2,4),'ZData',T03(3,4));
%     set(endeffector,'XData',T04(1,4),'YData',T04(2,4),'ZData',T04(3,4)); 
%     pause(0.02)
    i = i + 1;
end
Theta_pinvTP = Theta;

t = zeros(n+1,1);
X = zeros(n+1,1);
y = zeros(n+1,1);
Theta = zeros(3,n+1);
lambda = zeros(3,n+1);
U = zeros(3,n);
G = zeros(3,n);
G_pre = zeros(3,n);
Scale = zeros(3,n);
Scale_pre = zeros(3,n);
DFDQT = zeros(3,3);
DLDQ = zeros(3,1);
Thetadot = zeros(3,n+1);
cost_pinv = zeros(n+1,1);
cost_CGM = zeros(n+1,1);
sum_cost = zeros(101,1);
alpha = 0.2;
beta = 1;

i = 1;
count = 1;
t(1) = 0;            % Time  [sec]
Theta(1,1) = a1;    % Theta_1  [deg]
Theta(2,1) = a2;    % Theta_2  [deg]
Theta(3,1) = a3;    % Theta_3  [deg]
X(1)  = xi;          % Cartesian componet X [cm]
Y(1)  = yi;          % Cartesian componet Y [cm]
cost_pinv(1,1) = cost(L1,L2,L3,Theta(1,1),Theta(2,1),Theta(3,1));
cost_pinvTP(1,1) = cost(L1,L2,L3,Theta(1,1),Theta(2,1),Theta(3,1));
cost_CGM(1,1) = cost(L1,L2,L3,Theta(1,1),Theta(2,1),Theta(3,1));
i = i + 1;

%same calculation and results as pinv
while i < n + 2
    %[pinvJ]= pinvJ_2D(L1,L2,L3,a1,a2,a3);
    %dq = pinvJ*dx;
    
    Thetadot(:,i) = Qdot(L1,L2,L3,Theta(1,i-1),Theta(2,i-1),Theta(3,i-1),Xdot,Ydot,0,0,0);
    Theta(1,i) = Theta(1,i-1) + Thetadot(1,i);
    Theta(2,i) = Theta(2,i-1) + Thetadot(2,i);
    Theta(3,i) = Theta(3,i-1) + Thetadot(3,i);
    cost_pinv(i,1) = cost(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
%     [T01,T02,T03,T04] = Forward(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
    % Update the simulation plot using matlab function set
%     set(link1,'XData',[T01(1,4), T02(1,4)],'YData',[T01(2,4), T02(2,4)],'ZData',[T01(3,4), T02(3,4)]);
%     set(link2,'XData',[T02(1,4), T03(1,4)],'YData',[T02(2,4), T03(2,4)],'ZData',[T02(3,4), T03(3,4)]);
%     set(link3,'XData',[T03(1,4), T04(1,4)],'YData',[T03(2,4), T04(2,4)],'ZData',[T03(3,4), T04(3,4)]);
%     
%     set(joint1,'XData',T01(1,4),'YData',T01(2,4),'ZData',T01(3,4));    
%     set(joint2,'XData',T02(1,4),'YData',T02(2,4),'ZData',T02(3,4));
%     set(joint3,'XData',T03(1,4),'YData',T03(2,4),'ZData',T03(3,4));
%     set(endeffector,'XData',T04(1,4),'YData',T04(2,4),'ZData',T04(3,4)); 
%     pause(0.02)
    i = i + 1;
end

%store pinv data
Theta_pinv = Theta;
sum_cost(1,1) = sum(cost_pinv);




%timer start
tic

%calculation of lambda
i = n;
while i > 0
    DFDQT = dfdqT(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i),Xdot,Ydot,U(1,i),U(2,i),U(3,i));
    DLDQ = dldq(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
    lambda(:,i) = DLDQ + DFDQT * lambda(:,i+1) + lambda(:,i+1);
    i = i - 1;
end

%calculation of U
i = 1;
while i < n + 1
    DFDUT = dfduT(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i),Xdot,Ydot,U(1,i),U(2,i),U(3,i));
    DLDU = dldu(Thetadot(:,i),DFDUT);
    G(:,i) = DLDU + DFDUT * lambda(:,i+1);
    i = i + 1;
end

Scale = -G;
U = U + alpha*Scale;
%store G and Scale
G_pre = G;
Scale_pre = Scale;




%calculation of Theta for next iteration
i = 2;
while i < n + 2
    %[pinvJ]= pinvJ_2D(L1,L2,L3,a1,a2,a3);
    %dq = pinvJ*dx;
    Thetadot(:,i) = Qdot(L1,L2,L3,Theta(1,i-1),Theta(2,i-1),Theta(3,i-1),Xdot,Ydot,U(1,i-1),U(2,i-1),U(3,i-1));
    
    Theta(1,i) = Theta(1,i-1) + Thetadot(1,i);
    Theta(2,i) = Theta(2,i-1) + Thetadot(2,i);
    Theta(3,i) = Theta(3,i-1) + Thetadot(3,i);
    
    cost_CGM(i,1) = cost(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
    
%     [T01,T02,T03,T04] = Forward(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
%     % Update the simulation plot using matlab function set
%     set(link1,'XData',[T01(1,4), T02(1,4)],'YData',[T01(2,4), T02(2,4)],'ZData',[T01(3,4), T02(3,4)]);
%     set(link2,'XData',[T02(1,4), T03(1,4)],'YData',[T02(2,4), T03(2,4)],'ZData',[T02(3,4), T03(3,4)]);
%     set(link3,'XData',[T03(1,4), T04(1,4)],'YData',[T03(2,4), T04(2,4)],'ZData',[T03(3,4), T04(3,4)]);
%     
%     set(joint1,'XData',T01(1,4),'YData',T01(2,4),'ZData',T01(3,4));    
%     set(joint2,'XData',T02(1,4),'YData',T02(2,4),'ZData',T02(3,4));
%     set(joint3,'XData',T03(1,4),'YData',T03(2,4),'ZData',T03(3,4));
%     set(endeffector,'XData',T04(1,4),'YData',T04(2,4),'ZData',T04(3,4)); 
%     pause(0.01)
    i = i + 1;
end

%count of CGM iteration
count = count + 1;
sum_cost(count,1) = sum(cost_CGM);

while count < 101 
    
    %calculation of lambda
    i = n;
    while i > 0
        DFDQT = dfdqT(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i),Xdot,Ydot,U(1,i),U(2,i),U(3,i));
        DLDQ = dldq(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
        lambda(:,i) = DLDQ + DFDQT * lambda(:,i+1) + lambda(:,i+1);
        i = i - 1;
    end

    %calculation of U
    i = 1;
    while i < n + 1
        DFDUT = dfduT(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i),Xdot,Ydot,U(1,i),U(2,i),U(3,i));
        DLDU = dldu(Thetadot(:,i),DFDUT);
        G(:,i) = DLDU + DFDUT * lambda(:,i+1);
        i = i + 1;
    end
    
    beta = sum(sum(G.^2))/(sum(sum(G_pre.^2)));
    Scale = -G + beta*Scale_pre;
    U = U + alpha*Scale;
    %store G and Scale
    G_pre = G;
    Scale_pre = Scale;
    
    %count of CGM iteration
    count = count + 1;

    %calculation of Theta for next iteration
    i = 2;
    while i < n + 2
        %[pinvJ]= pinvJ_2D(L1,L2,L3,a1,a2,a3);
        %dq = pinvJ*dx;
        Thetadot(:,i) = Qdot(L1,L2,L3,Theta(1,i-1),Theta(2,i-1),Theta(3,i-1),Xdot,Ydot,U(1,i-1),U(2,i-1),U(3,i-1));

        Theta(1,i) = Theta(1,i-1) + Thetadot(1,i);
        Theta(2,i) = Theta(2,i-1) + Thetadot(2,i);
        Theta(3,i) = Theta(3,i-1) + Thetadot(3,i);
        
        cost_CGM(i,1) = cost(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
        
%         [T01,T02,T03,T04] = Forward(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
%         % Update the simulation plot using matlab function set
%         set(link1,'XData',[T01(1,4), T02(1,4)],'YData',[T01(2,4), T02(2,4)],'ZData',[T01(3,4), T02(3,4)]);
%         set(link2,'XData',[T02(1,4), T03(1,4)],'YData',[T02(2,4), T03(2,4)],'ZData',[T02(3,4), T03(3,4)]);
%         set(link3,'XData',[T03(1,4), T04(1,4)],'YData',[T03(2,4), T04(2,4)],'ZData',[T03(3,4), T04(3,4)]);
%         
%         set(joint1,'XData',T01(1,4),'YData',T01(2,4),'ZData',T01(3,4));    
%         set(joint2,'XData',T02(1,4),'YData',T02(2,4),'ZData',T02(3,4));
%         set(joint3,'XData',T03(1,4),'YData',T03(2,4),'ZData',T03(3,4));
%         set(endeffector,'XData',T04(1,4),'YData',T04(2,4),'ZData',T04(3,4)); 
%         pause(0.01)
        i = i + 1;
    end
    sum_cost(count,1) = sum(cost_CGM);
    if (sum_cost(count-1,1) > sum_cost(count,1) && (sum_cost(count-1,1) - sum_cost(count,1))/sum_cost(count-1,1) < 8e-5) || (count > 20 &&sum_cost(count,1) > sum_cost(count-1,1))
%         break
    end
end
count
toc

figure('Name','Arm animation','NumberTitle','off')
% Plots of the Arm
hold on
link0=plot3([0,Ti01(1,4)],[0,Ti01(2,4)],[0,Ti01(3,4)],'-r','LineWidth',5);
link1=plot3([Ti01(1,4), Ti02(1,4)],[Ti01(2,4), Ti02(2,4)],[Ti01(3,4), Ti02(3,4)],'-b','LineWidth',5);
link2=plot3([Ti02(1,4), Ti03(1,4)],[Ti02(2,4), Ti03(2,4)],[Ti02(3,4), Ti03(3,4)],'-g','LineWidth',5);
link3=plot3([Ti03(1,4), Ti04(1,4)],[Ti03(2,4), Ti04(2,4)],[Ti03(3,4), Ti04(3,4)],'-r','LineWidth',5);
joint1=plot3(Ti01(1,4),Ti01(2,4),Ti01(3,4),'-ko','MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63],'MarkerSize',10);
joint2=plot3(Ti02(1,4),Ti02(2,4),Ti02(3,4),'-ko','MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63],'MarkerSize',10);
joint3=plot3(Ti03(1,4),Ti03(2,4),Ti03(3,4),'-ko','MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63],'MarkerSize',10);
endeffector=plot3(Ti04(1,4),Ti04(2,4),Ti04(3,4),'-ko','LineWidth',5,'MarkerSize',5);

link0p=plot3([0,Ti01(1,4)],[0,Ti01(2,4)],[0,Ti01(3,4)],'color',1-0.25*(1-[1,0,0]),'LineWidth',5);
link1p=plot3([Ti01(1,4), Ti02(1,4)],[Ti01(2,4), Ti02(2,4)],[Ti01(3,4), Ti02(3,4)],'color',1-0.25*(1-[0,1,0]),'LineWidth',5);
link2p=plot3([Ti02(1,4), Ti03(1,4)],[Ti02(2,4), Ti03(2,4)],[Ti02(3,4), Ti03(3,4)],'color',1-0.25*(1-[0,0,1]),'LineWidth',5);
link3p=plot3([Ti03(1,4), Ti04(1,4)],[Ti03(2,4), Ti04(2,4)],[Ti03(3,4), Ti04(3,4)],'color',1-0.25*(1-[1,0,0]),'LineWidth',5);
joint1p=plot3(Ti01(1,4),Ti01(2,4),Ti01(3,4),'-ko','MarkerEdgeColor',1-0.25*(1-[0,0,0]),'MarkerFaceColor',1-0.25*(1-[.49 1 .63]),'MarkerSize',10);
joint2p=plot3(Ti02(1,4),Ti02(2,4),Ti02(3,4),'-ko','MarkerEdgeColor',1-0.25*(1-[0,0,0]),'MarkerFaceColor',1-0.25*(1-[.49 1 .63]),'MarkerSize',10);
joint3p=plot3(Ti03(1,4),Ti03(2,4),Ti03(3,4),'-ko','MarkerEdgeColor',1-0.25*(1-[0,0,0]),'MarkerFaceColor',1-0.25*(1-[.49 1 .63]),'MarkerSize',10);
endeffectorp=plot3(Ti04(1,4),Ti04(2,4),Ti04(3,4),'-o','color',1-0.25*(1-[0,0,0]),'LineWidth',5,'MarkerSize',5);

link0pTP=plot3([0,Ti01(1,4)],[0,Ti01(2,4)],[0,Ti01(3,4)],'color',1-0.5*(1-[1,0,0]),'LineWidth',5);
link1pTP=plot3([Ti01(1,4), Ti02(1,4)],[Ti01(2,4), Ti02(2,4)],[Ti01(3,4), Ti02(3,4)],'color',1-0.5*(1-[0,1,0]),'LineWidth',5);
link2pTP=plot3([Ti02(1,4), Ti03(1,4)],[Ti02(2,4), Ti03(2,4)],[Ti02(3,4), Ti03(3,4)],'color',1-0.5*(1-[0,0,1]),'LineWidth',5);
link3pTP=plot3([Ti03(1,4), Ti04(1,4)],[Ti03(2,4), Ti04(2,4)],[Ti03(3,4), Ti04(3,4)],'color',1-0.5*(1-[1,0,0]),'LineWidth',5);
joint1pTP=plot3(Ti01(1,4),Ti01(2,4),Ti01(3,4),'-ko','MarkerEdgeColor',1-0.5*(1-[0,0,0]),'MarkerFaceColor',1-0.5*(1-[.49 1 .63]),'MarkerSize',10);
joint2pTP=plot3(Ti02(1,4),Ti02(2,4),Ti02(3,4),'-ko','MarkerEdgeColor',1-0.5*(1-[0,0,0]),'MarkerFaceColor',1-0.5*(1-[.49 1 .63]),'MarkerSize',10);
joint3pTP=plot3(Ti03(1,4),Ti03(2,4),Ti03(3,4),'-ko','MarkerEdgeColor',1-0.5*(1-[0,0,0]),'MarkerFaceColor',1-0.5*(1-[.49 1 .63]),'MarkerSize',10);
endeffectorpTP=plot3(Ti04(1,4),Ti04(2,4),Ti04(3,4),'-o','color',1-0.5*(1-[0,0,0]),'LineWidth',5,'MarkerSize',5);

% x=-300:100:300; % simulation plane 
% y=x;
% [x,y]=meshgrid(x,y);
% z=x*0;
% surf(x,y,z);
% % shading interp
% alpha(0.6)
grid on;
axis([-120 120 -120 120 -120 120 -120 120])
view(2);

% Plots of points of interest on the system:
initial=plot3(Ti04(1,4),Ti04(2,4),Ti04(3,4),'-ro','LineWidth',5,'MarkerSize',5);
goal=plot3(xg,yg,0,'-go','LineWidth',5,'MarkerSize',5);
title('ARM Animation'); xlabel('x, (cm)'); ylabel('y (cm)'); zlabel('z (cm)');
hold off;

% pause


i = 2;
while i < n + 2
    %[pinvJ]= pinvJ_2D(L1,L2,L3,a1,a2,a3);
    %dq = pinvJ*dx;
%     Thetadot(:,i) = Qdot(L1,L2,L3,Theta(1,i-1),Theta(2,i-1),Theta(3,i-1),Xdot,Ydot,U(1,i-1),U(2,i-1),U(3,i-1));
% 
%     Theta(1,i) = Theta(1,i-1) + Thetadot(1,i);
%     Theta(2,i) = Theta(2,i-1) + Thetadot(2,i);
%     Theta(3,i) = Theta(3,i-1) + Thetadot(3,i);

%     cost_CGM(i,1) = cost(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i

    [T01p,T02p,T03p,T04p] = Forward(L1,L2,L3,Theta_pinv(1,i),Theta_pinv(2,i),Theta_pinv(3,i));
    set(link1p,'XData',[T01p(1,4), T02p(1,4)],'YData',[T01p(2,4), T02p(2,4)],'ZData',[T01p(3,4), T02p(3,4)]);
    set(link2p,'XData',[T02p(1,4), T03p(1,4)],'YData',[T02p(2,4), T03p(2,4)],'ZData',[T02p(3,4), T03p(3,4)]);
    set(link3p,'XData',[T03p(1,4), T04p(1,4)],'YData',[T03p(2,4), T04p(2,4)],'ZData',[T03p(3,4), T04p(3,4)]);
    set(joint1p,'XData',T01p(1,4),'YData',T01p(2,4),'ZData',T01p(3,4));    
    set(joint2p,'XData',T02p(1,4),'YData',T02p(2,4),'ZData',T02p(3,4));
    set(joint3p,'XData',T03p(1,4),'YData',T03p(2,4),'ZData',T03p(3,4));
    set(endeffectorp,'XData',T04p(1,4),'YData',T04p(2,4),'ZData',T04p(3,4)); 

    [T01pTP,T02pTP,T03pTP,T04pTP] = Forward(L1,L2,L3,Theta_pinvTP(1,i),Theta_pinvTP(2,i),Theta_pinvTP(3,i));
    set(link1pTP,'XData',[T01pTP(1,4), T02pTP(1,4)],'YData',[T01pTP(2,4), T02pTP(2,4)],'ZData',[T01pTP(3,4), T02pTP(3,4)]);
    set(link2pTP,'XData',[T02pTP(1,4), T03pTP(1,4)],'YData',[T02pTP(2,4), T03pTP(2,4)],'ZData',[T02pTP(3,4), T03pTP(3,4)]);
    set(link3pTP,'XData',[T03pTP(1,4), T04pTP(1,4)],'YData',[T03pTP(2,4), T04pTP(2,4)],'ZData',[T03pTP(3,4), T04pTP(3,4)]);
    set(joint1pTP,'XData',T01pTP(1,4),'YData',T01pTP(2,4),'ZData',T01pTP(3,4));    
    set(joint2pTP,'XData',T02pTP(1,4),'YData',T02pTP(2,4),'ZData',T02pTP(3,4));
    set(joint3pTP,'XData',T03pTP(1,4),'YData',T03pTP(2,4),'ZData',T03pTP(3,4));
    set(endeffectorpTP,'XData',T04pTP(1,4),'YData',T04pTP(2,4),'ZData',T04pTP(3,4)); 
    
    [T01,T02,T03,T04] = Forward(L1,L2,L3,Theta(1,i),Theta(2,i),Theta(3,i));
    % Update the simulation plot using matlab function set
    set(link1,'XData',[T01(1,4), T02(1,4)],'YData',[T01(2,4), T02(2,4)],'ZData',[T01(3,4), T02(3,4)]);
    set(link2,'XData',[T02(1,4), T03(1,4)],'YData',[T02(2,4), T03(2,4)],'ZData',[T02(3,4), T03(3,4)]);
    set(link3,'XData',[T03(1,4), T04(1,4)],'YData',[T03(2,4), T04(2,4)],'ZData',[T03(3,4), T04(3,4)]);
    set(joint1,'XData',T01(1,4),'YData',T01(2,4),'ZData',T01(3,4));    
    set(joint2,'XData',T02(1,4),'YData',T02(2,4),'ZData',T02(3,4));
    set(joint3,'XData',T03(1,4),'YData',T03(2,4),'ZData',T03(3,4));
    set(endeffector,'XData',T04(1,4),'YData',T04(2,4),'ZData',T04(3,4)); 
    
    pause(0.04)
    i = i + 1;
end

figure('Name','cost value vs iteration','NumberTitle','off');
plot(sum_cost(1:count,1));



    
    
    
%     
%     [T01n,T02n,T03n,T04n,T05n,T06n,T07n] = Forward(L1,L2,L4,L6,a1,a2,a3,a4,a5,a6,a7,d1,d2,d3,d4,d5,d6);
%     
%     
% %     for j=1:7
% %         if q(j)<qmin(j) || q(j)>qmax(j)
% %             disp(['WARNING: q',num2str(j),' is out']);
% %         end
% %     end
%     
%     
%     % Recalculate velocity of end-effector by using dx
%     Xdot  = dx(1); %[cm/sec]
%     Ydot  = dx(2); %[cm/sec]
%     Zdot  = dx(3); %[cm/sec]
%     adot  = dx(4); %[rad/sec] 
%     Bdot  = dx(5); %[rad/sec] 
%     rdot  = dx(6); %[rad/sec]  
%     dx = [Xdot;Ydot;Zdot;adot;Bdot;rdot];
% 
%     % Save the time for plotting
%     t(i+1) = t(i) + dt;
% 
%     % Update the Trasforment matrixes 
%     T07=T07n;
%     T06=T06n;
%     T05=T05n;
%     T04=T04n;
%     T03=T03n;
%     T02=T02n;
%     T01=T01n;
%     % Update the simulation plot using matlab function set
%     set(link1,'XData',[T01(1,4), T02(1,4)],'YData',[T01(2,4), T02(2,4)],'ZData',[T01(3,4), T02(3,4)]);
%     set(link2,'XData',[T02(1,4), T03(1,4)],'YData',[T02(2,4), T03(2,4)],'ZData',[T02(3,4), T03(3,4)]);
%     set(link3,'XData',[T03(1,4), T04(1,4)],'YData',[T03(2,4), T04(2,4)],'ZData',[T03(3,4), T04(3,4)]);
%     set(link4,'XData',[T04(1,4), T05(1,4)],'YData',[T04(2,4), T05(2,4)],'ZData',[T04(3,4), T05(3,4)]);
%     set(link5,'XData',[T05(1,4), T06(1,4)],'YData',[T05(2,4), T06(2,4)],'ZData',[T05(3,4), T06(3,4)]);
%     set(link6,'XData',[T06(1,4), T07(1,4)],'YData',[T06(2,4), T07(2,4)],'ZData',[T06(3,4), T07(3,4)]);
%     
%     set(joint1,'XData',T01(1,4),'YData',T01(2,4),'ZData',T01(3,4));    
%     set(joint2,'XData',T02(1,4),'YData',T02(2,4),'ZData',T02(3,4));
%     set(joint3,'XData',T03(1,4),'YData',T03(2,4),'ZData',T03(3,4));
%     set(joint4,'XData',T04(1,4),'YData',T04(2,4),'ZData',T04(3,4));
%     set(joint5,'XData',T05(1,4),'YData',T05(2,4),'ZData',T05(3,4));
%     set(joint6,'XData',T06(1,4),'YData',T06(2,4),'ZData',T06(3,4));
%     set(endeffector,'XData',T07(1,4),'YData',T07(2,4),'ZData',T07(3,4)); 
%     
%     % pause to allow plotting
%     pause(0.01);   
%     
%     % Save the three active joints rates theta_dot
%     ThetaDot(:,i) = dq;
%     % Save the three active joint angles theta_1,Theta_2, Theta_3,
%     % Theta_4,Theta_5,Theta_6
%     Theta(:,i+1) = [a1;a2;a3;a4;a5;a6;a7];
%     % Save the Jacobian matrix determinant detJ
%     DETJo(i+1) = detJo;
%     DETJ(i+1) = detJ;
%     SF(i+1) = sf;
% 
%     %Save the three Cartesian components for the end-effector [x y phi]
%     X(i+1) = T07(1,4);
%     Y(i+1) = T07(2,4);
%     Z(i+1) = T07(3,4);
%     
%     [ai, Bi, ri] = T2rpy(T07);
%     a(i+1) = ai;
%     B(i+1) = Bi;
%     r(i+1) = ri;
%     
%     i = i+1;
%         
% 
% % Find the Jacobian at last time value to make all the variables equal
% % length
% 
% % Transformation Matrix of Goal Position 
% disp('Transformation Matrix of Goal Position')
% [T01,T02,T03,T04,T05,T06,T07] = Forward(L1,L2,L4,L6,a1,a2,a3,a4,a5,a6,a7,d1,d2,d3,d4,d5,d6);
% T07
% Tg
% %Tt = traj(1, T07, Tg, n+1);
% %     Tt(:,:,1)
% %     Tt(:,:,n)
% %     Tt(:,:,n+1)
% a_end=a(i)
% B_end=B(i)
% r_end=r(i)
% [J] = Jacobian_3D(L1,L2,L4,L6,a1,a2,a3,a4,a5,a6,a7,d1,d2,d3,d4,d5,d6);
% [ag, Bg, rg] = T2rpy(Tg);
% d = [T07(1,4); T07(2,4); T07(3,4); a(i); B(i); r(i)]
% 
% % dq in every step
% ThetaDot(:,i) = [0;0;0;0;0;0;0];
% 
% %DETJ(i+1) = DETJ(i);
% [T01,T02,T03,T04,T05,T06,T07] = Forward(L1,L2,L4,L6,a1,a2,a3,a4,a5,a6,a7,d1,d2,d3,d4,d5,d6);
% 
% figure('Name','parameters vs time','NumberTitle','off')
% % Plot joint velocity vs time
% subplot(2,2,1), plot(t,ThetaDot)
% title('joint velocity vs time');
% 
% % Plot joint angles vs time
% subplot(2,2,2), plot(t,Theta)
% title('joint angles vs time');
% legend('Theta(1)','Theta(2)','Theta(3)','Theta(4)','Theta(5)','Theta(6)','Theta(7)','Location','East');
% 
% % Plot position vs time
% subplot(2,2,3), plot(t,X,'b',t,Y,'g',t,Z,'r')
% title('position vs time');
% legend('X','Y','Z','Location','East');
% 
% subplot(2,2,4), plot(t,r,'b',t,B,'g',t,a,'r')
% title('orientation vs time');
% legend('a','B','r','Location','East');
% 
% figure('Name','SF','NumberTitle','off')
% plot(t,SF ,'r' );
% %grid on; 
% title('Sf vs Time'); xlabel('time, (sec)'); ylabel(' Sf '); 
% 
% figure('Name','DetJo','NumberTitle','off')
% plot( t, DETJo, 'b', t, DETJo, '-m');
% %grid on; 
% title('Manipulability Measure vs Time'); xlabel('time, (sec)'); ylabel('Manipulability Measure');
% 
% figure('Name','DetJ','NumberTitle','off')
% plot( t, DETJ, 'b', t, DETJ, '-m');
% %grid on; 
% title('Manipulability Measure vs Time'); xlabel('time, (sec)'); ylabel('Manipulability Measure'); 
