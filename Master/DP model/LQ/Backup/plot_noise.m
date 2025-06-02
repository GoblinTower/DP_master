figure;
hold on;
plot(t_array, x_est_array(7,:));
plot(t_array, x_est_array(8,:));
plot(t_array, x_est_array(9,:));
xlabel('t [s]');
title('Noise terms');
legend(['w1', 'w2', 'w3']);
hold off;