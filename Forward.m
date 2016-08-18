function [T01,T02,T03,T04]= Forward(L1,L2,L3,theta1,theta2,theta3) 

[T01] = Transform(0, 0,0,theta1);
[T12] = Transform(0,L1, 0,theta2);
[T23] = Transform(0,L2,0,theta3);
[T34] = Transform(0,L3,0,0);
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
