%% Close all Clear all
close all
clear all
clc

%% Robot prism 3m
pr1 = [4.7936, 0.7133, -1.1588];
pr2 = [4.9039, 0.5273, -1.1567];
pr3 = [4.7644, 0.5646, -1.1601];

pr = [pr1', pr2', pr3'];



%% Gate prism 3m
p1 = [4.834, 0.9204, -1.1524];
p2 = [4.945, 0.7433, -1.1495];
p3 = [4.8056, 0.7702, -1.1546];

p = [p1', p2', p3'];


%% Base
Vgp1 = [-0.0713284, 0.0768284, 0.0209];
Vgp2 = [0.0713284, -0.0768284, 0.0209];
Vgp3 = [-0.0713284, -0.0768284, 0.0209];

b = [Vgp1', Vgp2', Vgp3'];

%% plots
% scatter3(b(1,:), b(2,:), b(3,:))
% figure
% scatter3(p(1,:), p(2,:), p(3,:))
% figure
% scatter3(pr(1,:), pr(2,:), p(3,:))


%% math
dist = sqrt((pr1(1)-p1(1))^2+(pr1(2)-p1(2))^2+(pr1(3)-p1(3))^2)
dist = sqrt((pr2(1)-p2(1))^2+(pr2(2)-p2(2))^2+(pr2(3)-p2(3))^2)
dist = sqrt((pr3(1)-p3(1))^2+(pr3(2)-p3(2))^2+(pr3(3)-p3(3))^2)

centroid_gate = sum(p,2)/3
centroid_robot = sum(pr,2)/3

dist = sqrt((centroid_robot(1)-centroid_gate(1))^2 + (centroid_robot(2)-centroid_gate(2))^2 + (centroid_robot(3)-centroid_gate(3))^2)

v1_prime = pr-centroid_robot
v2_prime = p-centroid_gate

M = v1_prime*v2_prime'

S_xx = M(1,1)
S_xy = M(1,2)
S_xz = M(1,3)
S_yx = M(2,1)
S_yy = M(2,2)
S_yz = M(2,3)
S_zx = M(3,1)
S_zy = M(3,2)
S_zz = M(3,3)
N = [S_xx+S_yy+S_zz, S_yz-S_zy, S_zx-S_xz, S_xy-S_yx; 
    S_yz-S_zy, S_xx-S_yy-S_zz, S_xy+S_yx, S_zx+S_xz; 
    S_zx-S_xz, S_xy+S_yx, -S_xx+S_yy-S_zz, S_yz+S_zy; 
    S_xy-S_yx, S_zx+S_xz, S_yz+S_zy, -S_xx-S_yy+S_zz]

[V,D] = eig(N)
max_val = max(max(D))

quat = quaternion(V(1,4), V(2,4), V(3,4), V(4,4))
eul = quat2eul(quat,'XYZ')
eul_deg = eul.*180/pi

