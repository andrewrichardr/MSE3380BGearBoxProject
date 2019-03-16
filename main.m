%% Inputs
clear
clc

%All in standard SI units
OperatingVoltage = 40
TorqueConstant = 8.474E-3
VoltageConstant = 1125
ArmatureResistance = 0.072
reqOutputSpeed = 575
reqOutputTorque = 55
Efficiency = 90/100



%% Motor
%point 1:
NoLoadSpeed = VoltageConstant*OperatingVoltage;
NoLoadTorque = 0;
%point 2:
StallTorque = TorqueConstant*OperatingVoltage/ArmatureResistance;
StallSpeed = 0;

%determine Torque-speed graph
x = [StallSpeed, NoLoadSpeed]
y = [StallTorque, NoLoadTorque]
p = polyfit(x,y,1)

f1 = figure('Renderer', 'painters', 'Position', [10 10 1200 300])
subplot(1,3,1)
plot([StallSpeed:NoLoadSpeed], polyval(p,[StallSpeed:NoLoadSpeed]))
hold on
title("Motor Torque-Speed")
xlabel("Speed (rpm)")
ylabel("Torque (N.m)")
hold off

%% Motor Power
power = [0]
for x = [StallSpeed:NoLoadSpeed]
    power = [power, polyval(p,x)*x*pi/30];
end
power = power(2:end);
outputPower = power.*Efficiency;
subplot(1,3,2)
plot(power)
hold on
plot(outputPower)
title("Power")
xlabel("Motor Speed (rpm)")
ylabel("Power (Watts)")
legend("Motor Output","System Output")
xlim([StallSpeed, NoLoadSpeed])
hold off
[peakPwr,peakPwrSpeed] = findpeaks(power)


%% Ratios

ratio = round(peakPwrSpeed/reqOutputSpeed);
stage1Ratio = ratio^0.5;
stage2Ratio = ratio^0.5;

outputTorque = [0]
outputSpeed = [0]

for x = [StallSpeed:NoLoadSpeed]
   outputSpeed = [outputSpeed, x/ratio];
   outputTorque = [outputTorque, outputPower(x+1)*ratio*30/(pi*x)];
end
subplot(1,3,3)
plot(outputSpeed, outputTorque)
hold on
plot(575,55,'*')
title("Drill Torque-Speed")
xlabel("Drill Speed (rpm)")
ylabel("Drill Torque (N.m)")
legend("Drill Output", "Given Requirement")
hold off

%%

