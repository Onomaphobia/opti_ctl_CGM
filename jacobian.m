clear all
close all
clc

syms L1 L2 L3 theta1 theta2 theta3

[T01] = Transform(0, 0,0,theta1);
[T12] = Transform(0,L1, 0,theta2);
[T23] = Transform(0,L2,0,theta3);
[T34] = Transform(0,L3,0,0);
T = T01*T12*T23*T34;
T = simplify(T)

J(1,1) = diff(T(1,4),theta1);
J(1,2) = diff(T(1,4),theta2);
J(1,3) = diff(T(1,4),theta3);
J(2,1) = diff(T(2,4),theta1);
J(2,2) = diff(T(2,4),theta2);
J(2,3) = diff(T(2,4),theta3);
J = simplify(J)
