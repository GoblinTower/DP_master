% Test find_closest_angle function

% Test 1
array = deg2rad([10, 40, -270, 10, 270]);
shortest_array_angles = find_closest_angle(array);

disp('Array input and expected array');
expected_answer = [10, 40, 90, 10, -90];
disp(expected_answer);
disp(round(rad2deg(shortest_array_angles)));

% Test 2
array = deg2rad([10, -10, -10, -60, 40, 100, 250, 10, 160, 10, ...
    170, -20, 60, -60, -200, 10, -150, -250, 20, 270, 180, 10, 270]);
shortest_array_angles = find_closest_angle(array);

disp('Array input and expected array');
expected_answer = [10, -10, -10, -60, 40, 100, 250, 370, 520, 370, 530, 700, 780, 660, 520, 370, 210, 110, 20, -90, -180, -350, -450];
disp(expected_answer);
disp(round(rad2deg(shortest_array_angles)));
