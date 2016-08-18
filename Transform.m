function [T] = Transform(alpha,a,d,theta)

s = sin(theta);
c = cos(theta);
sa = sin(alpha);
ca = cos(alpha);
T = [c -s 0 a;
     s*ca c*ca -sa -sa*d;
     s*sa c*sa ca ca*d;
     0 0 0 1];
 