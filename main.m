clear
clc

%% Inputs

%All in standard SI units unless noted
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
DiametralPitch = [8, 10, 12, 16, 20, 24, 32, 48]        %teeth/inch
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
MotorLine = polyfit(points(:,1),points(:,2),polynomialDegree);

f1 = figure('Renderer', 'painters', 'Position', [10 10 1200 300])
subplot(1,3,1)
plot([StallSpeed:NoLoadSpeed], polyval(MotorLine,[StallSpeed:NoLoadSpeed]))
hold on
MotorLineE = Efficiency.*MotorLine
plot([StallSpeed:NoLoadSpeed], polyval(MotorLineE,[StallSpeed:NoLoadSpeed]))
title("Motor Torque-Speed")
xlabel("Motor Speed (rpm)")
ylabel("Motor Torque (N.m)")
legend("Theoretical Torque", "Real Torque")
hold off

%% Gear Ratio Target (minimization)
syms r
eqn = MotorLineE(1)*reqOutputSpeed*r^2 + MotorLineE(2)*r == reqOutputTorque;
soln = double(solve(eqn,r))

ratioTarget = soln(1)

TargetLine = [MotorLineE(1)*ratioTarget^2, MotorLineE(2)*ratioTarget]

subplot(1,3,2)
plot([StallSpeed/ratioTarget: NoLoadSpeed/ratioTarget], polyval(TargetLine,[StallSpeed/ratioTarget: NoLoadSpeed/ratioTarget]))
hold on
plot(reqOutputSpeed, reqOutputTorque,'*')
title(["Drill Torque-Speed (Target), m = ", ratioTarget])
xlabel("Drill Speed (rpm)")
ylabel("Drill Torque (N.m)")
legend("Drill Output", "Given Requirement")
hold off


%% Gears

%pinions
P1 = ceil((2*k*(m1+(m1^2+(1-2*m1)*(sin(PressureAngle1))^2)^0.5))/((1+2*m1)*(sin(PressureAngle1))^2));
P2 = ceil((2*k*(m2+(m2^2+(1-2*m2)*(sin(PressureAngle2))^2)^0.5))/((1+2*m2)*(sin(PressureAngle2))^2));
%gears
N1l = floor(((P1^2)*(sin(PressureAngle1)^2)-4*k^2)/(4*k-2*P1*sin(PressureAngle1)^2));
N2l = floor(((P1^2)*(sin(PressureAngle2)^2)-4*k^2)/(4*k-2*P2*sin(PressureAngle2)^2));

if(m1*P1 > N1l)
    N1 = N1l;   %Gear Ratio m is too large
    disp("Ratio m1 Too Large")
else
    N1 = floor(m1*P1);
end

if(m2*P2 > N2l);
    N2 = N2l;    %Gear Ratio m is too large
    disp("Ratio m2 Too Large")
else
    N2 = floor(m2*P2);
end

m1actual = N1/P1;
m2actual = N2/P2;
ratio = m1actual*m2actual;

RatioTable = table(m1, m1actual, P1, N1, m2, m2actual, P2, N2, ratio, ratioTarget)


%% Gear Ratio (Overall)

outputTorque = [0];
outputSpeed = [0];

RealLine = [MotorLineE(1)*ratio^2, MotorLineE(2)*ratio]

for x = [StallSpeed:NoLoadSpeed]
   outputSpeed = [outputSpeed, x/ratio];
   outputTorque = [outputTorque, polyval(RealLine,x/ratio)];
end
outputSpeed = outputSpeed(2:end);
outputTorque = outputTorque(2:end);
subplot(1,3,3)
plot(outputSpeed, outputTorque)
hold on
plot(reqOutputSpeed,reqOutputTorque,'*')
title(["Drill Torque-Speed (Actual) m = ", ratio])
xlabel("Drill Speed (rpm)")
ylabel("Drill Torque (N.m)")
legend("Drill Output", "Given Requirement")
hold off

%% Shaft Torque and Speed Estimation
%{
Motor - P1  : Shaft A
N1 - P2     : Shaft B
N2 - Chuck  : Shaft C
%}

%Shaft C
CmaxTorque = outputTorque(1);
CopTorque = reqOutputTorque;
CmaxSpeed = outputSpeed(length(outputSpeed));
CopSpeed = reqOutputSpeed;
%Shaft B
BmaxTorque = CmaxTorque/m2;
BopTorque = CopTorque/m2;
BmaxSpeed = CmaxSpeed/m2;
BopSpeed = CopSpeed/m2;
%Shaft A
AmaxTorque = BmaxTorque/m1;
AopTorque = BopTorque/m1;
AmaxSpeed = BmaxSpeed/m1;
AopSpeed = BopSpeed/m1;

ShaftTorqueEstimation = table(AopTorque, AmaxTorque, BopTorque, BmaxTorque, CopTorque, CmaxTorque)
ShaftSpeedEstimation = table(AopSpeed, AmaxSpeed, BopSpeed, BmaxSpeed, CopSpeed, CmaxSpeed)

%% Gear Forces and Diameter

nomForce1 = [0];
nomForce2 = [0];
maxForce1 = [0];
maxForce2 = [0];
P1diameter = [0];
N1diameter = [0];
P2diameter = [0];
N2diameter = [0];


for x = DiametralPitch
DiametralPitch1 = x/0.0254;
DiametralPitch2 = x/0.0254;  %Teeth/inch -> teeth/meter

P1dia = P1/DiametralPitch1;
N1dia= N1/DiametralPitch1;
P2dia= P2/DiametralPitch2;
N2dia= N2/DiametralPitch2;

P1diameter = [P1diameter, P1dia];
N1diameter = [N1diameter, N1dia];
P2diameter = [P2diameter, P2dia];
N2diameter = [N2diameter, N2dia];

%Nominal Torques
nomForce1 = [nomForce1, AopTorque/(P1dia/2)];
nomForce2 = [nomForce2, BopTorque/(P2dia/2)];
%Maximum Torques
maxForce1 = [maxForce1, AmaxTorque/(P1dia/2)];
maxForce2 = [maxForce2, BmaxTorque/(P2dia/2)];

end

nomForce1 = nomForce1(2:end);
nomForce2 = nomForce2(2:end);
maxForce1 = maxForce1(2:end);
maxForce2 = maxForce2(2:end);
P1diameter = P1diameter(2:end);
N1diameter = N1diameter(2:end);
P2diameter = P2diameter(2:end);
N2diameter = N2diameter(2:end);


f2 = figure('Renderer', 'painters', 'Position', [10 10 900 400])
subplot(1,2,1)
bar([1:length(DiametralPitch)], maxForce1)
hold on
bar([1:length(DiametralPitch)], nomForce1)
title("Contact Forces 1")
legend("maximum","nominal")
set(gca, 'XTick', [1:length(DiametralPitch)])
set(gca, 'XTickLabel', {DiametralPitch})
xlabel("Diametral Pitch")
ylabel("Force (N)")
hold off
subplot(1,2,2)
bar([1:length(DiametralPitch)], maxForce2)
hold on
bar([1:length(DiametralPitch)], nomForce2)
title("Contact Forces 2")
legend("maximum","nominal")
set(gca, 'XTick', [1:length(DiametralPitch)])
set(gca, 'XTickLabel', {DiametralPitch})
xlabel("Diametral Pitch")
ylabel("Force (N)")
hold off

DiametralPitch = transpose(DiametralPitch);
maxForce1 = transpose(maxForce1);
nomForce1 = transpose(nomForce1);
maxForce2 = transpose(maxForce2);
nomForce2 = transpose(nomForce2);
P1diameter = transpose(P1diameter).*100; %meters -> centimeters
N1diameter = transpose(N1diameter).*100;
P2diameter = transpose(P2diameter).*100;
N2diameter = transpose(N2diameter).*100;

ForceDiameterTable = table(DiametralPitch, maxForce1, nomForce1, P1diameter, N1diameter, maxForce2, nomForce2, P2diameter, N2diameter)

