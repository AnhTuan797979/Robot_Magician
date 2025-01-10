% CODE ĐỘNG HỌC NGHỊCH GIẢI 4 BỘ NGHIỆM CỦA ROBOT 3 BẬC TỰ DO 

% Khai báo các biến ký hiệu
syms q1 q2 q3 Px Py Pz l1 l2 l3

% Đặt các giá trị cố định cho l1, l2, l3
% l1 = 160;
% l2 = 200;
% l3 = 250;
% Px=200;
% Py=0;
% Pz=20;


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




