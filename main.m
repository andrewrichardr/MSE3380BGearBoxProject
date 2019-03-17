clear
clc

%% Inputs

%All in standard SI units
OperatingVoltage = 40           %V
TorqueConstant = 8.474E-3       %N.m/A
VoltageConstant = 1125          %RPM/V
ArmatureResistance = 0.072      %Ohms
reqOutputSpeed = 575            %RPM
reqOutputTorque = 55            %N.m
Efficiency = 90/100             
Duty = 2000                     %Hours

%Gears
PressureAngle1 = 20*pi/180      %Rad
PressureAngle2 = 20*pi/180      %Rad
DiametralPitch = 1
k = 1                           %Teeth Depth (1 = full)
m1 = 5                          %Gear Ratio Stage 1
m2 = 4                          %Gear Ratio Stage 2


%% Motor Torque-Speed
%point 1:
NoLoadSpeed = VoltageConstant*OperatingVoltage;
NoLoadTorque = 0;
%point 2:
StallTorque = TorqueConstant*OperatingVoltage/ArmatureResistance;
StallSpeed = 0;

%determine Torque-speed graph
points = [StallSpeed, StallTorque;
          NoLoadSpeed, NoLoadTorque];
polynomialDegree = length(points)-1;
p = polyfit(points(:,1),points(:,2),polynomialDegree);

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

%% Gears

%pinions
P1 = ceil((2*k*(m1+(m1^2+(1-2*m1)*(sin(PressureAngle1))^2)^0.5))/((1+2*m1)*(sin(PressureAngle1))^2));
P2 = ceil((2*k*(m2+(m2^2+(1-2*m2)*(sin(PressureAngle2))^2)^0.5))/((1+2*m2)*(sin(PressureAngle2))^2));
%gears
N1l = floor(((P1^2)*(sin(PressureAngle1)^2)-4*k^2)/(4*k-2*P1*sin(PressureAngle1)^2));
N2l = floor(((P1^2)*(sin(PressureAngle2)^2)-4*k^2)/(4*k-2*P2*sin(PressureAngle2)^2));

if(m1*P1 > N1l)
    N1 = N1l;   %Gear Ratio m is too large
else
    N1 = floor(m1*P1);
end

if(m2*P2 > N2l);
    N2 = N2l;    %Gear Ratio m is too large
else
    N2 = floor(m2*P2);
end

m1actual = N1/P1;
m2actual = N2/P2;
ratio = m1actual*m2actual;

RatioTable = table(m1, m1actual, P1, N1, m2, m2actual, P2, N2, ratio)


%% Gear Ratio (Overall)

outputTorque = [0];
outputSpeed = [0];

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


