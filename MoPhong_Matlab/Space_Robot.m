% Tham số của robot
l1 = 150;
l2 = 200;
l3 = 200;


% Số mẫu trong mỗi khớp
n_samples = 20;

% Giới hạn góc của các khớp (theo độ)
theta1_min = -90; theta1_max = 90;
theta2_min = -30; theta2_max = 90;
theta3_min = -30; theta3_max = 70;

% Chuyển đổi độ sang radian
theta1_min = deg2rad(theta1_min); theta1_max = deg2rad(theta1_max);
theta2_min = deg2rad(theta2_min); theta2_max = deg2rad(theta2_max);
theta3_min = deg2rad(theta3_min); theta3_max = deg2rad(theta3_max);

% Tạo các mảng góc cho mỗi khớp
theta1 = linspace(theta1_min, theta1_max, n_samples);
theta2 = linspace(theta2_min, theta2_max, n_samples);
theta3 = linspace(theta3_min, theta3_max, n_samples);

% Tạo các lưới góc khớp
[q1, q2, q3] = ndgrid(theta1, theta2, theta3);

% Tính toán tọa độ điểm cuối của robot (x, y, z)
X = cos(q1).*(l3.*cos(q2 + q3) + l2.*sin(q2));
Y = sin(q1).*(l3.*cos(q2 + q3) + l2.*sin(q2));
Z = l1 - l3.*sin(q2 + q3) + l2.*cos(q2);
  % Z chỉ phụ thuộc vào góc TH2 và TH3

% Vẽ không gian làm việc
figure;
scatter3(X(:), Y(:), Z(:),5, 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Không gian làm việc của robot 3 bậc tự do');
grid on;
axis equal;
