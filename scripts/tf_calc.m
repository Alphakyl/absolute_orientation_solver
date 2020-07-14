function [outputArg1,outputArg2] = tf_calc(pl,pr)

centroid_left = sum(pl,2)/3;
centroid_right = sum(pr,2)/3;

figure
hold on
grid on
scatter3(pl(1,:), pl(2,:), pl(3,:))
scatter3(pr(1,:), pr(2,:), pr(3,:))
scatter3(centroid_left(1), centroid_left(2), centroid_left(3), 'c*')
scatter3(centroid_right(1), centroid_right(2), centroid_right(3), 'r*')
view(3)
title('Positions from Leica')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
legend('Left', 'Right', 'Centroid L', 'Centroid R')

raw_dist = sqrt((centroid_right(1)-centroid_left(1))^2 + (centroid_right(2)-centroid_left(2))^2 + (centroid_right(3)-centroid_left(3))^2)
offset = centroid_left - centroid_right

v1_prime = pr-centroid_right;
v2_prime = pl-centroid_left;

figure
hold on
grid on
scatter3(v2_prime(1,:), v2_prime(2,:), v2_prime(3,:))
scatter3(v1_prime(1,:), v1_prime(2,:), v1_prime(3,:))
view(3)
title('Positions after removing centroid')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
legend('Left prism set', 'Right prism set')


M = v1_prime*v2_prime';

S_xx = M(1,1);
S_xy = M(1,2);
S_xz = M(1,3);
S_yx = M(2,1);
S_yy = M(2,2);
S_yz = M(2,3);
S_zx = M(3,1);
S_zy = M(3,2);
S_zz = M(3,3);
N = [S_xx+S_yy+S_zz, S_yz-S_zy, S_zx-S_xz, S_xy-S_yx; 
    S_yz-S_zy, S_xx-S_yy-S_zz, S_xy+S_yx, S_zx+S_xz; 
    S_zx-S_xz, S_xy+S_yx, -S_xx+S_yy-S_zz, S_yz+S_zy; 
    S_xy-S_yx, S_zx+S_xz, S_yz+S_zy, -S_xx-S_yy+S_zz];

[V,D] = eig(N);
[max_val, max_val_id] = max(max(D));

quat = quaternion(V(1,max_val_id), V(2,max_val_id), V(3,max_val_id), V(4,max_val_id));
eul = quat2eul(quat,'XYZ');
eul_deg = eul.*180/pi
rotm = quat2rotm(quat);
end

