function [tf_mat] = tf_calc(pl,pr)


% Find Centroids
centroid_left = sum(pl,2)/3
centroid_right = sum(pr,2)/3

% Plot initial positions
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

% Calculate raw distance and raw offset of centroids
raw_dist = sqrt((centroid_right(1)-centroid_left(1))^2 + (centroid_right(2)-centroid_left(2))^2 + (centroid_right(3)-centroid_left(3))^2)
offset = centroid_left - centroid_right


% Remove centroids
vr_prime = pr-centroid_right
vl_prime = pl-centroid_left

% Plot points after removing centroids
figure
hold on
grid on
scatter3(vr_prime(1,:), vr_prime(2,:), vr_prime(3,:))
scatter3(vl_prime(1,:), vl_prime(2,:), vl_prime(3,:))
view(3)
title('Positions after removing centroid')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis([-0.15 0.15 -0.15 0.15 -0.15 0.15])
legend('Left prism set', 'Right prism set')

% Create M 
M = vl_prime*vr_prime';

% Pull sums from M
S_xx = M(1,1);
S_xy = M(1,2);
S_xz = M(1,3);
S_yx = M(2,1);
S_yy = M(2,2);
S_yz = M(2,3);
S_zx = M(3,1);
S_zy = M(3,2);
S_zz = M(3,3);

% Create N
N = [S_xx+S_yy+S_zz, S_yz-S_zy, S_zx-S_xz, S_xy-S_yx; 
    S_yz-S_zy, S_xx-S_yy-S_zz, S_xy+S_yx, S_zx+S_xz; 
    S_zx-S_xz, S_xy+S_yx, -S_xx+S_yy-S_zz, S_yz+S_zy; 
    S_xy-S_yx, S_zx+S_xz, S_yz+S_zy, -S_xx-S_yy+S_zz];

% Find eigen vecotrs and heighest eigen value
[V,D] = eig(N);
[max_val, max_val_id] = max(max(D));

% Get rotation (eigen vector with the highest eigen value)
quat = quaternion(V(1,max_val_id), V(2,max_val_id), V(3,max_val_id), V(4,max_val_id));
eul = quat2eul(quat,'XYZ');
eul_deg = eul.*180/pi
rotm = quat2rotm(quat);

% Turn rotation matrix into transformation matrix
rot_mat = [rotm, zeros(3,1)];
rot_mat = [rot_mat; zeros(1,3), 1];

% Find Scale
SL = 0;
SR = 0;
for col=1:length(pl)
    SL = SL + vl_prime(:,col)'*vl_prime(:,col);
    SR = SR + vr_prime(:,col)'*vr_prime(:,col);
end
display(SR)
display(SL)
s = sqrt(SR/SL)

translation_with_dummy_1 = [centroid_right; 1] - s * rot_mat * [centroid_left; 1];
translation = translation_with_dummy_1(1:3)

translation_mat = eye(4);
translation_mat(1:3,4)=translation;

tf_mat = rot_mat*translation_mat;
end

