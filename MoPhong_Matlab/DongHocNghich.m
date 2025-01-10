% CODE ĐỘNG HỌC NGHỊCH GIẢI 4 BỘ NGHIỆM CỦA ROBOT 3 BẬC TỰ DO 

% Khai báo các biến ký hiệu
syms q1 q2 q3 Px Py Pz

% Đặt các giá trị cố định cho l1, l2, l3
l1 = 150;
l2 = 200;
l3 = 200;

Px = 200;
Py = 0;
Pz = 350;

% Tính 2 giá trị của q1
q1_solution1 = atan2(Py, Px);
q1_solution2 = atan2(-Py, -Px);

% Tính toán cho trường hợp q1 = atan2(Py, Px)
disp('Trường hợp 1: q1 = atan2(Py, Px)');

% Thay q1_solution1 vào phương trình còn lại và giải cho q2, q3
eq1 = l1 - l3 * sin(q2 + q3) + l2 * cos(q2) == Pz;   % Phương trình cho Pz
eq2 = l3 * cos(q2 + q3) + l2 * sin(q2) == sqrt(Px^2 + Py^2);  % Tính r = sqrt(Px^2 + Py^2)

% Sử dụng hàm solve để giải q2 và q3 cho q1_solution1
solutions1 = solve([eq1, eq2], [q2, q3]);

% Lấy nghiệm từ solve
q2_solution1 = simplify(solutions1.q2);  % Nghiệm của q2 cho q1_solution1
q3_solution1 = simplify(solutions1.q3);  % Nghiệm của q3 cho q1_solution1

% Hiển thị kết quả cho q1_solution1
disp('Giá trị của q1 là:');
disp(q1_solution1);

disp('Giá trị của q2 là:');
disp(q2_solution1);

disp('Giá trị của q3 là:');
disp(q3_solution1);

% Tính toán cho trường hợp q1 = atan2(-Py, -Px)
disp('Trường hợp 2: q1 = atan2(-Py, -Px)');

% Sử dụng lại phương trình eq1 và eq2, giải cho q2, q3 với q1_solution2
solutions2 = solve([eq1, eq2], [q2, q3]);

% Lấy nghiệm từ solve cho q1_solution2
q2_solution2 = simplify(solutions2.q2);  % Nghiệm của q2 cho q1_solution2
q3_solution2 = simplify(solutions2.q3);  % Nghiệm của q3 cho q1_solution2

% Hiển thị kết quả cho q1_solution2
disp('Giá trị của q1 là:');
disp(q1_solution2);

disp('Giá trị của q2 là:');
disp(q2_solution2);

disp('Giá trị của q3 là:');
disp(q3_solution2);


% Vẽ robot dựa trên nghiệm
figure;

% Trường hợp 1: q1 = atan2(Py, Px)
subplot(2, 2, 1);
plot_robot(q1_solution1, q2_solution1(1), q3_solution1(1), l1, l2, l3);
title('Trường hợp 1: q1 = atan2(Py, Px)');

% Kiểm tra nếu tồn tại nhiều nghiệm, vẽ thêm trường hợp cho q1_solution2
if length(q2_solution1) > 1 && length(q3_solution1) > 1
    subplot(2, 2, 2);
    plot_robot(q1_solution1, q2_solution1(2), q3_solution1(2), l1, l2, l3);
    title('Trường hợp 1: Nghiệm 2');
end

% Trường hợp 2: q1 = atan2(-Py, -Px)
subplot(2, 2, 3);
plot_robot(q1_solution2, q2_solution2(1), q3_solution2(1), l1, l2, l3);
title('Trường hợp 2: q1 = atan2(-Py, -Px)');

% Kiểm tra nếu tồn tại nhiều nghiệm, vẽ thêm trường hợp cho q1_solution2
if length(q2_solution2) > 1 && length(q3_solution2) > 1
    subplot(2, 2, 4);
    plot_robot(q1_solution2, q2_solution2(2), q3_solution2(2), l1, l2, l3);
    title('Trường hợp 2: Nghiệm 2');
end

