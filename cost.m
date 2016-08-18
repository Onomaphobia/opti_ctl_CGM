function[cost] = cost(L1,L2,L3,theta1,theta2,theta3)

cost = 1/(sin(conj(theta2))*abs(L1)^2*abs(L2)^2*sin(theta2) + 2*sin(conj(theta3))*abs(L2)^2*abs(L3)^2*sin(theta3) + L3*sin(conj(theta2))*abs(L1)^2*conj(L2)*cos(theta2)*sin(theta3) + L3*sin(conj(theta2))*abs(L1)^2*conj(L2)*cos(theta3)*sin(theta2) + L1*sin(conj(theta3))*abs(L3)^2*conj(L2)*cos(theta2)*sin(theta3) + L1*sin(conj(theta3))*abs(L3)^2*conj(L2)*cos(theta3)*sin(theta2) + 2*cos(conj(theta2))*sin(conj(theta3))*abs(L1)^2*abs(L3)^2*cos(theta2)*sin(theta3) + 2*cos(conj(theta2))*sin(conj(theta3))*abs(L1)^2*abs(L3)^2*cos(theta3)*sin(theta2) + 2*cos(conj(theta3))*sin(conj(theta2))*abs(L1)^2*abs(L3)^2*cos(theta2)*sin(theta3) + 2*cos(conj(theta3))*sin(conj(theta2))*abs(L1)^2*abs(L3)^2*cos(theta3)*sin(theta2) + L2*cos(conj(theta2))*sin(conj(theta3))*abs(L1)^2*conj(L3)*sin(theta2) + L2*cos(conj(theta3))*sin(conj(theta2))*abs(L1)^2*conj(L3)*sin(theta2) + L2*cos(conj(theta2))*sin(conj(theta3))*abs(L3)^2*conj(L1)*sin(theta3) + L2*cos(conj(theta3))*sin(conj(theta2))*abs(L3)^2*conj(L1)*sin(theta3))^(1/2);
