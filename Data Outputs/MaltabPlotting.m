close
clc
clear

filename1 = 'MotorPowers1.txt';
filename2 = 'MotorPowers2.txt';
filename3 = 'MotorPowers3.txt';
filename4 = 'MotorPowers4.txt';

FID1 = fopen(filename1, 'r');
FID2 = fopen(filename2, 'r');
FID3 = fopen(filename3, 'r');
FID4 = fopen(filename4, 'r');


data = fscanf(FID1, '%f', [3, inf]);
I = data(1,:);
DR1 = data(2,:);
DL1 = data(3,:);

data = fscanf(FID2, '%f', [3, inf]);
DR2 = data(2,:);
DL2 = data(3,:);

data = fscanf(FID3, '%f', [3, inf]);
DR3 = data(2,:);
DL3 = data(3,:);

data = fscanf(FID4, '%f', [3, inf]);
DR4 = data(2,:);
DL4 = data(3,:);

for i = 1 : length(I)
    DR(i) = (DR1(i) + DR2(i) + DR3(i) + DR4(i)) / 4;
    DL(i) = (DL1(i) + DL2(i) + DL3(i) + DL4(i)) / 4;
end

figure(1);
subplot(1,4,1);
plot(I,DR1,'r', I, DL1, 'b');
legend('DR', 'DL');
ylabel('Distance (counts)');
xlabel('Power)');
grid on
grid minor

subplot(1,4,2);
plot(I,DR2,'r', I, DL2, 'b');
legend('DR', 'DL');
ylabel('Distance (counts)');
xlabel('Power)');
grid on
grid minor

subplot(1,4,3);
plot(I,DR3,'r', I, DL3, 'b');
legend('DR', 'DL');
ylabel('Distance (counts)');
xlabel('Power)');
grid on
grid minor

subplot(1,4,4);
plot(I,DR1,'r', I, DL, 'b');
legend('DR', 'DL');
ylabel('Distance (counts)');
xlabel('Power)');
grid on
grid minor

