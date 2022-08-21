close
clc
clear


FIDs(1)= fopen('MotorPowersLow1.txt', 'r');
FIDs(2) = fopen('MotorPowersLow2.txt', 'r');
FIDs(3) = fopen('MotorPowersLow3.txt', 'r');



for i = 1: 3
    data = fscanf(FIDs(i), '%f', [3, inf]);
    ILow = data(1,:);
    DRLow(i,:) = data(2,:);
    DLLow(i,:) = data(3,:);
end




DRAverageLow = mean(DRLow);
DLAverageLow = mean(DLLow);



figure(1);
plot(ILow,DRAverageLow,'r', ILow, DLAverageLow, 'b', ILow+2,DLAverageLow,'k');
legend('DR', 'DL','DL Adjusted');
ylabel('Distance (counts)');
xlabel('Power)');
grid on
grid minor





