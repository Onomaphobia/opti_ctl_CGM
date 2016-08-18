function [J] = Jacobian_2D(L1,L2,L3, theta1,theta2,theta3)

 
J=[ - L2*sind(theta1 + theta2) - L1*sind(theta1) - L3*sind(theta1 + theta2 + theta3), - L2*sind(theta1 + theta2) - L3*sind(theta1 + theta2 + theta3), -L3*sind(theta1 + theta2 + theta3);
      L2*cosd(theta1 + theta2) + L1*cosd(theta1) + L3*cosd(theta1 + theta2 + theta3),   L2*cosd(theta1 + theta2) + L3*cosd(theta1 + theta2 + theta3),  L3*cosd(theta1 + theta2 + theta3)];
 