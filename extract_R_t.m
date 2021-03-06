function [M_rot2_1,M_trans2_1] = extract_R_t(E)

[U,S,V] = svd(E);
R1 = U*[0,-1,0;1,0,0;0,0,1]*V'
R2 = U*[0,1,0;-1,0,0;0,0,1]*V'
x1 = U*[0,1,0;-1,0,0;0,0,0]*U'
x2 = -U*[0,1,0;-1,0,0;0,0,0]*U'
t1 = [x1(3,2),x1(1,3),x1(2,1)];
t2 = [x2(3,2),x2(1,3),x2(2,1)];
M_rot2_1 = x1;
M_trans2_1 = t1;