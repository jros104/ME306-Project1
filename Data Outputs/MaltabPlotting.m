filename = 'CircleV3Run1.txt';

FID = fopen(filename, 'r');

data = fscanf(FID, '%f', [5, inf]);
T = data(1,:);
TL = data(2,:);
TR = data(3,:);
posL = data(4,:);
posR = data(5,:);

figure(1);
subplot(1,2,1);
plot(T,TL,'r', T, posL, 'b');
legend('Target Left', 'posL');
ylabel('Distance (counts)');
xlabel('Time (s)');
grid on
grid minor
subplot(1,2,2);
plot(T,TR,'r', T, posR, 'b');
legend('Target Right', 'posR');
ylabel('Distance (counts)');
xlabel('Time (s)');
grid on
grid minor