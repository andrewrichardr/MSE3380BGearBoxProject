clear
clc
resloution = 0.001;
%% Inputs

%All in standard SI units unless noted
OperatingVoltage = 40                                               %V
TorqueConstant = 8.474E-3                                           %N.m/A
VoltageConstant = 1125                                              %RPM/V
ArmatureResistance = 0.072                                          %Ohms
reqOutputSpeed = 575                                                %RPM
reqOutputTorque = 55                                                %N.m
Efficiency = 90/100             
Duty = 2000                                                         %Hours

%Gear Specs
PressureAngle = 20*pi/180                                           %Rad
FaceWidth = [1.5, 1.25, 1, 0.75, 0.5, 0.25, 0.188, 0.125]           %inch
DiametralPitch = [8, 10, 12, 16, 20, 24, 32, 48]                    %teeth/inch
DiametralPitchSelected = 20;
k = 1                                                               %Teeth Depth (1 = full)

%Number of teeth variables:
P1 = 16
N1 = 70
P2 = 16
N2 = 70

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

%fit a polynomial to the two data points
MotorLine = polyfit(points(:,1),points(:,2),polynomialDegree);

%generate figure
f1 = figure('Renderer', 'painters', 'Position', [10 10 1200 300])
subplot(1,3,1)
plot([StallSpeed:NoLoadSpeed], polyval(MotorLine,[StallSpeed:NoLoadSpeed]))
hold on
title("Motor Torque-Speed")
xlabel("Motor Speed (rpm)")
ylabel("Motor Torque (N.m)")
hold off

%% Gear Ratio Target (minimization)

%setup equation to determine optimal ratio
syms r
eqn = MotorLine(1)*reqOutputSpeed*r^2 + MotorLine(2)*r == reqOutputTorque;
soln = double(solve(eqn,r))

%solve equation for tearget ratio
ratioTarget = soln(1);

%determine line for the target ratio
TargetLine = [MotorLine(1)*ratioTarget^2, MotorLine(2)*ratioTarget];

%generate figure
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

m1actual = N1/P1;
m2actual = N2/P2;
ratio = m1actual*m2actual;

%limits:
%pinions
P1l = ceil((2*k*(m1actual+(m1actual^2+(1-2*m1actual)*(sin(PressureAngle))^2)^0.5))/((1+2*m1actual)*(sin(PressureAngle))^2));
P2l = ceil((2*k*(m2actual+(m2actual^2+(1-2*m2actual)*(sin(PressureAngle))^2)^0.5))/((1+2*m2actual)*(sin(PressureAngle))^2));

%gear Limits
N1l = floor(((P1l^2)*(sin(PressureAngle)^2)-4*k^2)/(4*k-2*P1l*sin(PressureAngle)^2));
N2l = floor(((P1l^2)*(sin(PressureAngle)^2)-4*k^2)/(4*k-2*P2l*sin(PressureAngle)^2));

%generate ratio table
TRatio = table(P1, P1l, N1, N1l, m1actual, P2, P2l, N2, N2l, m2actual, ratio)

%old ratio determiner
%{
if(m1*P1l > N1l)
    N1 = N1l;   %Gear Ratio m is too large
    disp("Ratio m1 Too Large, increasing P1")
    P1l = P1l+1;
    N1 = floor(m1*P1l);
else
    N1 = floor(m1*P1l);
end

if(m2*P2 > N2l);
    N2 = N2l;    %Gear Ratio m is too large
    disp("Ratio m2 Too Large, increasing P2")
    P2 = P2+1;
    N2 = floor(m2*P2);
else
    N2 = floor(m2*P2);
end

%}

%% Gear Ratio (Overall)

outputTorque = [0];
outputSpeed = [0];

%Adjust line with ratio
RealLine = [MotorLine(1)*ratio^2, MotorLine(2)*ratio];

for x = [StallSpeed:NoLoadSpeed]
   outputSpeed = [outputSpeed, x/ratio];
   outputTorque = [outputTorque, polyval(RealLine,x/ratio)];
end
outputSpeed = outputSpeed(2:end);
outputTorque = outputTorque(2:end);

%generate figure
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

outputPower = reqOutputSpeed*reqOutputTorque*pi/30 ;
intermediatePower = outputPower/Efficiency^0.5;
inputPower = intermediatePower/Efficiency^0.5;
efficiency = outputPower/inputPower;

%Shaft C
SpeedC = reqOutputSpeed;
TorqueC = reqOutputTorque;
%Shaft B
SpeedB = SpeedC*m2actual;
TorqueB = intermediatePower/(SpeedB*pi/30);
%Shaft A
SpeedA = SpeedB*m1actual;
TorqueA = inputPower/(SpeedA*pi/30);

%Table for torque and speed estimations
TTorqueSpeedEstimation = table(TorqueA, SpeedA, TorqueB, SpeedB, TorqueC, SpeedC)

%table for power estimations
TPower = table(inputPower, intermediatePower, outputPower, efficiency)

%% Operating Gear Forces, Torques and Diameters

%initialize arrays
Force1t = [0];
Force1r = [0];
Force2t = [0];
Force2r = [0];
P1diameter = [0];
N1diameter = [0];
P2diameter = [0];
N2diameter = [0];

for x = DiametralPitch
    x = x/0.0254;  %Teeth/inch -> teeth/meter
    P1dia = P1/x; %Teeth / (Teeth/meter) = meter
    N1dia = N1/x;
    P2dia = P2/x;
    N2dia = N2/x;
    
    %record diameter
    P1diameter = [P1diameter, P1dia];
    N1diameter = [N1diameter, N1dia];
    P2diameter = [P2diameter, P2dia];
    N2diameter = [N2diameter, N2dia];
    
    %Transmitted Load: Pinion 1 -> Gear 1 EQN 13-36
    Wt1 = (60000*inputPower*10^-3)/(pi*(P1dia*1000)*SpeedA);
    %Radial Load: Pinion 1 -> Gear 1
    Wr1 = tan(PressureAngle)*Wt1;
    
    %Transmitted Load: Pinion 2 -> Gear 2
    Wt2 = (60000*outputPower*10^-3)/(pi*(N2dia*1000)*SpeedC);
    %Radial Load: Pinion 2 -> Gear 2
    Wr2 = tan(PressureAngle)*Wt2;
    
    %Record Forces
    Force1t = [Force1t, Wt1*1000]; %kN -> N
    Force1r = [Force1r, Wr1*1000];
    Force2t = [Force2t, Wt2*1000];
    Force2r = [Force2r, Wr2*1000];
end

%Prepare arrays for table
Force1t = transpose(Force1t(2:end));
Force2t = transpose(Force2t(2:end));
Force1r = transpose(Force1r(2:end));
Force2r = transpose(Force2r(2:end));
P1diameter = transpose(P1diameter(2:end)).*100; %meters -> centimeters;
N1diameter = transpose(N1diameter(2:end)).*100; %meters -> centimeters;
P2diameter = transpose(P2diameter(2:end)).*100; %meters -> centimeters;
N2diameter = transpose(N2diameter(2:end)).*100; %meters -> centimeters;

%Table with Forces and Diameters
TForceDiameter = table(transpose(DiametralPitch), Force1t, Force1r, P1diameter, N1diameter, Force2t, Force2r, P2diameter, N2diameter)

%{

Not confident that this is correct

%Wt, Tangential Forces
Force1t = [0];
Force2t = [0];
%Wr, Radial Forces
Force1r = [0];
Force2r = [0];
%Gear Diameters
P1diameter = [0];
N1diameter = [0];
P2diameter = [0];
N2diameter = [0];
%Shaft Torques
AT = [0];
BT = [0];
CT = [0];

for x = DiametralPitch
DiametralPitch1 = x/0.0254;
DiametralPitch2 = x/0.0254;  %Teeth/inch -> teeth/meter

P1dia = P1/DiametralPitch1; %Teeth / (Teeth/meter) = meter
N1dia= N1/DiametralPitch1;
P2dia= P2/DiametralPitch2;
N2dia= N2/DiametralPitch2;

%Logging diameters for the table below
P1diameter = [P1diameter, P1dia];
N1diameter = [N1diameter, N1dia];
P2diameter = [P2diameter, P2dia];
N2diameter = [N2diameter, N2dia];

%Tangential Forces
Ctorque = CopTorque;
Wt2 = Ctorque/(cos(PressureAngle2)*N2dia/2);
Btorque = (Wt2*P2dia/2)/cos(PressureAngle2);
Wt1 = Btorque/(cos(PressureAngle1)*N1dia/2);
Atorque = (Wt1*P1dia/2)/cos(PressureAngle1);
Force2t = [Force2t, Wt2];
Force1t = [Force1t, Wt1];
%Radial Forces
Force1r = [Force1r, tan(PressureAngle1)*Wt1];
Force2r = [Force2r, tan(PressureAngle2)*Wt2];
%Shaft Torques
AT = [AT, Atorque];
BT = [BT, Btorque];
CT = [CT, Ctorque];

end

Force1t = Force1t(2:end);
Force2t = Force2t(2:end);
Force1r = Force1r(2:end);
Force2r = Force2r(2:end);

P1diameter = P1diameter(2:end);
N1diameter = N1diameter(2:end);
P2diameter = P2diameter(2:end);
N2diameter = N2diameter(2:end);

AT = AT(2:end);
BT = BT(2:end);
CT = CT(2:end);

DiametralPitch = transpose(DiametralPitch);
Force1t = transpose(Force1t);
Force2t = transpose(Force2t);
Force1r = transpose(Force1r);
Force2r = transpose(Force2r)
P1diameter = transpose(P1diameter).*100; %meters -> centimeters
N1diameter = transpose(N1diameter).*100;
P2diameter = transpose(P2diameter).*100;
N2diameter = transpose(N2diameter).*100;
AT = transpose(AT);
BT = transpose(BT);
CT = transpose(CT);

ForceTorqueTable = table(DiametralPitch, AT, Force1t, Force1r, BT,  Force2t, Force2r, CT)

DiameterTable = table(DiametralPitch, P1diameter, N1diameter, P2diameter, N2diameter)

%}

%% Stresses 
% (Calculated in USCS then converted to metric)

%Init Arrays
ContactPinion1 = [0];
BendingPinion1 = [0];
ContactGear1 = [0];
BendingGear1 = [0];
ContactPinion2 = [0];
BendingPinion2 = [0];
ContactGear2 = [0];
BendingGear2 = [0];

Pinion1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
Gear1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
Pinion2 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
Gear2 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];

for x = [1:length(DiametralPitch)]
 
    p1 = stresses(DiametralPitch(x), P1, SpeedA, FaceWidth(x), Force1t(x), m1actual, PressureAngle, SpeedA*Duty*60);%Speed*Duty*60 -> number of revs
    g1 = stresses(DiametralPitch(x), N1, SpeedB, FaceWidth(x), Force1t(x), m1actual, PressureAngle, SpeedB*Duty*60);
    p2 = stresses(DiametralPitch(x), P2, SpeedB, FaceWidth(x), Force2t(x), m2actual, PressureAngle, SpeedB*Duty*60);
    g2 = stresses(DiametralPitch(x), N2, SpeedC, FaceWidth(x), Force2t(x), m2actual, PressureAngle, SpeedC*Duty*60);
    
    Pinion1 = [Pinion1; p1];
    Gear1 = [Gear1; g1];
    Pinion2 = [Pinion2; p2];
    Gear2 = [Gear2; g2];

    if DiametralPitch(x) == DiametralPitchSelected
       PINION1 =  p1;
       GEAR1 =  g1;
       PINION2 =  p2;
       GEAR2 =  g2;
    end
    
end

Pinion1 = Pinion1(2:end,:);
Gear1 = Gear1(2:end,:);
Pinion2 = Pinion2(2:end,:);
Gear2 = Gear2(2:end,:);

%Appendix tables, DP varying
names = ["DP","ContactStress", "BendingStress", "ContactFOS", "BendingFOS", "Ko", "Kv", "Ks", "Km", "Kb", "Kt", "Kr", "Cf", "J", "I"];
Tpinion1 = array2table(Pinion1, 'VariableNames', names)
Tgear1 = array2table(Gear1, 'VariableNames', names)
Tpinion2 = array2table(Pinion2, 'VariableNames', names)
Tgear2 = array2table(Gear2, 'VariableNames', names)

%Prepare arrays for table
PINION1 = transpose(PINION1);
GEAR1 = transpose(GEAR1);
PINION2 = transpose(PINION2);
GEAR2 = transpose(GEAR2);

%Table for selected DP
Tselected = table(transpose(names), PINION1, GEAR1, PINION2, GEAR2)

%% Torque Calculation

Torque = zeros(3,1); %index 1 = Torque A, %index 2 = Torque B, %index 3 = Torque C,
Wt = zeros(4,1); %index 1 = P1, %index 2 = N1, %index 3 = P2, index 4 = N2
Wr = zeros(4,1);

Torque(3) = TorqueC;
Wt(4) = Torque(3)/(0.5*0.0254*N2/DiametralPitchSelected); %F = T/d, meters/inch* teeth / (teeth/inch)
Wr(4) = tan(PressureAngle)*Wt(4);

Torque(2) = TorqueB;
Wt(3) = Torque(2)/(0.5*0.0254*P2/DiametralPitchSelected); %F = T/d, meters/inch* teeth / (teeth/inch)
Wr(3) = tan(PressureAngle)*Wt(3);
Wt(2) = Torque(2)/(0.5*0.0254*N1/DiametralPitchSelected); %F = T/d, meters/inch* teeth / (teeth/inch)
Wr(2) = tan(PressureAngle)*Wt(2);

Torque(1) = TorqueA;
Wt(1) = Torque(1)/(0.5*0.0254*P1/DiametralPitchSelected); %F = T/d, meters/inch* teeth / (teeth/inch)
Wr(1) = tan(PressureAngle)*Wt(1);


TTorqueCalc = table(["Shaft A";"Shaft B";"Shaft C"], Torque)
TForceCalc = table(["Pinion 1";"Gear 1";"Pinion 2";"Gear 2"], Wt, Wr)

%% Shaft A BMD

%define Shaft A length and positions, units: meters
LengthA = 1.5*0.0254; %inch -> meter
BearingA2pos = 1.5*0.0254;
BearingA1pos = 0; %bearing supports shaft
Motor1pos = 0; %motor generates torque
Pinion1pos = 0.5*0.0254; %inch -> meter;

%generate system of equations for the shaft
% Variable naming convention: B1xy
% B = Bearing, P = Pinion, G = Gear, M = motor or chuck
% 1 = first bearing from the origin, 2 = second from origin...
% xy, xz, yz = reference plane
% Prad = pinion radius, Grad = gear radius
syms B1xy B1xz B2xy B2xz Pxy Pxz Gxy Gxz Myz 
syms B1Pos B2Pos PPos GPos MPos Prad Grad

%Pinion
Pxy = Wr(1);
Pxz = Wt(1);
PPos = Pinion1pos;
Prad = 0.5*0.0254*P1/DiametralPitchSelected;

%Gear
Gxy = 0;%No gear on shaft A
Gxz = 0;
GPos = 0;
Grad = 0;

%Motor
Myz = Torque(1);
Mpos = 0;

%Bearing Locations
B1Pos = BearingA1pos;
B2Pos = BearingA2pos;

% xy equations
SigmaFxy = B1xy + B2xy + Pxy + Gxy == 0
SigmaMxy = B1xy*B1Pos + B2xy*B2Pos + Pxy*PPos + Gxy*GPos == 0
 
% xz equations
SigmaFxz = B1xz + B2xz + Pxz + Gxz == 0
SigmaMxz = B1xz*B1Pos + B2xz*B2Pos + Pxz*PPos + Gxz*GPos == 0

% yz equations: gears are stacked in the y-direction
SigmaTyz = Myz + Pxz*Prad + Gxz*Grad == 0

%Solve system of equations
eqns = [SigmaFxy; SigmaMxy; SigmaFxz; SigmaMxz; SigmaTyz]
soln = solve(eqns)
soln = structfun(@double,soln)

%define bearing values
BearingA1 = [soln(1), soln(2)];
BearingA2 = [soln(3), soln(4)];

%Shear Force and Bending Moment Diagrams
padding = 0.2
f2 = figure()
hold on
title("Shaft A")
hold off
subplot(2,2,1)
ShearXY = Shear(LengthA, [B1Pos, PPos, B2Pos], [BearingA1(1) Pxy BearingA2(1)]);
plot(ShearXY(:,1)*1000, ShearXY(:,2))
hold on
title("XY Shear Force")
xlabel("Position (mm)")
ylabel("Shear Force (N)")
ylim([min(ShearXY(:,2))-range(ShearXY(:,2))*padding max(ShearXY(:,2))+range(ShearXY(:,2))*padding])
hold off
subplot(2,2,2)
ShearXZ = Shear(LengthA, [B1Pos, PPos, B2Pos], [BearingA1(2) Pxz BearingA2(2)]);
plot(ShearXZ(:,1)*1000, ShearXZ(:,2))
hold on
title("XZ Shear Force")
xlabel("Position (mm)")
ylabel("Shear Force (N)")
ylim([min(ShearXZ(:,2))-range(ShearXZ(:,2))*padding max(ShearXZ(:,2))+range(ShearXZ(:,2))*padding])
hold off
subplot(2,2,3)
BendingMomentXY = Moment(LengthA, [B1Pos, PPos, B2Pos], [BearingA1(1) Pxy BearingA2(1)]);
plot(BendingMomentXY(:,1)*1000, BendingMomentXY(:,2))
hold on
title("XY Bending Moment")
xlabel("Position (mm)")
ylabel("Bending Moment (N.m)")
ylim([min(BendingMomentXY(:,2))-range(BendingMomentXY(:,2))*padding max(BendingMomentXY(:,2))+range(BendingMomentXY(:,2))*padding])
hold off
subplot(2,2,4)
BendingMomentXZ = Moment(LengthA, [B1Pos, PPos, B2Pos], [BearingA1(2) Pxz BearingA2(2)]);
plot(BendingMomentXZ(:,1)*1000, BendingMomentXZ(:,2))
hold on
title("XZ Bending Moment")
xlabel("Position (mm)")
ylabel("Bending Moment (N.m)")
ylim([min(BendingMomentXZ(:,2))-range(BendingMomentXZ(:,2))*padding max(BendingMomentXZ(:,2))+range(BendingMomentXZ(:,2))*padding])
hold off

BearingA1 = transpose(BearingA1)
BearingA2 = transpose(BearingA2)
TShaftA = table(["XY"; "XZ"], BearingA1, [Pxy; Pxz], BearingA2)

%% Shaft B BMD

%define Shaft B length and positions, units: meters
LengthB = 1.5*0.0254; %inch -> meter
BearingB2pos = 1.5*0.0254;
BearingB1pos = 0; %bearing supports shaft
Gear1pos = 0.5*0.0254; %motor generates torque
Pinion2pos = 1*0.0254; %inch -> meter;

%generate system of equations for the shaft
% Variable naming convention: B1xy
% B = Bearing, P = Pinion, G = Gear, M = motor or chuck
% 1 = first bearing from the origin, 2 = second from origin...
% xy, xz, yz = reference plane
% Prad = pinion radius, Grad = gear radius
syms B1xy B1xz B2xy B2xz Pxy Pxz Gxy Gxz Myz 
syms B1Pos B2Pos PPos GPos MPos Prad Grad

%Pinion
Pxy = Wr(3);
Pxz = Wt(3);
PPos = Pinion2pos;
Prad = 0.5*0.0254*P2/DiametralPitchSelected;

%Gear
Gxy = Wr(2);
Gxz = Wt(2);
GPos = Gear1pos;
Grad = 0.5*0.0254*N1/DiametralPitchSelected;

%Motor/chuck
Myz = 0;
Mpos = 0;

%Bearing Locations
B1Pos = BearingB1pos;
B2Pos = BearingB2pos;

% xy equations
SigmaFxy = B1xy + B2xy + Pxy + Gxy == 0
SigmaMxy = B1xy*B1Pos + B2xy*B2Pos + Pxy*PPos + Gxy*GPos == 0
 
% xz equations
SigmaFxz = B1xz + B2xz + Pxz + Gxz == 0
SigmaMxz = B1xz*B1Pos + B2xz*B2Pos + Pxz*PPos + Gxz*GPos == 0

% yz equations: gears are stacked in the y-direction
SigmaTyz = Myz + Pxz*Prad + Gxz*Grad == 0

%Solve system of equations
eqns = [SigmaFxy; SigmaMxy; SigmaFxz; SigmaMxz; SigmaTyz]
soln = solve(eqns)
soln = structfun(@double,soln)

%define bearing values
BearingB1 = [soln(1), soln(2)];
BearingB2 = [soln(3), soln(4)];

%Shear Force and Bending Moment Diagrams
padding = 0.2
f3 = figure()
hold on
title("Shaft B")
hold off
subplot(2,2,1)
ShearXY = Shear(LengthB, [B1Pos, GPos, PPos, B2Pos], [BearingB1(1) Gxy Pxy BearingB2(1)]);
plot(ShearXY(:,1)*1000, ShearXY(:,2))
hold on
title("XY Shear Force")
xlabel("Position (mm)")
ylabel("Shear Force (N)")
ylim([min(ShearXY(:,2))-range(ShearXY(:,2))*padding max(ShearXY(:,2))+range(ShearXY(:,2))*padding])
hold off
subplot(2,2,2)
ShearXZ = Shear(LengthB, [B1Pos, GPos, PPos, B2Pos], [BearingB1(2) Gxz Pxz BearingB2(2)]);
plot(ShearXZ(:,1)*1000, ShearXZ(:,2))
hold on
title("XZ Shear Force")
xlabel("Position (mm)")
ylabel("Shear Force (N)")
ylim([min(ShearXZ(:,2))-range(ShearXZ(:,2))*padding max(ShearXZ(:,2))+range(ShearXZ(:,2))*padding])
hold off
subplot(2,2,3)
BendingMomentXY = Moment(LengthB, [B1Pos, GPos, PPos, B2Pos], [BearingB1(1) Gxy Pxy BearingB2(1)]);
plot(BendingMomentXY(:,1)*1000, BendingMomentXY(:,2))
hold on
title("XY Bending Moment")
xlabel("Position (mm)")
ylabel("Bending Moment (N.m)")
ylim([min(BendingMomentXY(:,2))-range(BendingMomentXY(:,2))*padding max(BendingMomentXY(:,2))+range(BendingMomentXY(:,2))*padding])
hold off
subplot(2,2,4)
BendingMomentXZ = Moment(LengthB, [B1Pos, GPos, PPos, B2Pos], [BearingB1(2) Gxz Pxz BearingB2(2)]);
plot(BendingMomentXZ(:,1)*1000, BendingMomentXZ(:,2))
hold on
title("XZ Bending Moment")
xlabel("Position (mm)")
ylabel("Bending Moment (N.m)")
ylim([min(BendingMomentXZ(:,2))-range(BendingMomentXZ(:,2))*padding max(BendingMomentXZ(:,2))+range(BendingMomentXZ(:,2))*padding])
hold off

BearingB1 = transpose(BearingB1)
BearingB2 = transpose(BearingB2)
TShaftB = table(["XY"; "XZ"], BearingB1, [Gxy; Gxz], [Pxy; Pxz], BearingB2)

%% Test
%{
L = 61
Fpos = [0 14 34 49 61]
F = [82.24 -80 -110 142.76 -35]
figure()
subplot(2,1,1)
plot(Shear(L, Fpos, F))
subplot(2,1,2)
plot(Moment(L, Fpos, F))
%}

%Shear Force Diagram Plotter: Tlength = length of shaft, Fpos = array of force
%positions (3 decimal places or fewer), F = array for force vectors (Transverse Direction Only)
function V = Shear(Tlength, FPos, F)
    RES = 10^-6; %
    size = ceil(Tlength/RES + 1);
    V = zeros(size, 2); %array which shear is plotted
    k = 0;
    for i = 1:length(FPos)
        k = k+1;
        v = F(k);
        for pos = 0:RES:Tlength
            index = int64((pos/RES)+1);  
            V(index, 1) = pos;            
            if(pos >= FPos(i))
                V(index, 2) = V(index, 2) + v;
            end
        end
    end
end

%Bending Moment Plotter: Tlength = length of shaft, Fpos = array of force
%positions (3 decimal places or fewer), F = array for force vectors (Transverse Direction Only)
function M = Moment(Tlength, FPos, F)
    RES = 10^-6;
    size = ceil(Tlength/RES + 1);
    M = zeros(size, 2); 
    k = 0;
    for i = 1:length(FPos)
        k = k+1;
        mx = F(k);
        for pos = 0:RES:Tlength
            index = int64((pos/RES)+1);
            M(index, 1) = pos;  
            if(pos >= FPos(i))
                M(index, 2) = M(index, 2) + mx*(pos - FPos(i));              
            end
        end
    end
end

%Bending Moment Plotter: Tlength = length of shaft, Fpos = array of force
%positions (3 decimal places or fewer), F = array for force vectors (Transverse Direction Only)
function T = TorquePlotter(Tlength, FPos, F)
    RES = 10^-6;
    size = ceil(Tlength/RES + 1);
    M = zeros(size, 2); 
    k = 0;
    for i = 1:length(FPos)
        k = k+1;
        mx = F(k)
        for pos = 0:RES:Tlength
            index = int64((pos/RES)+1);
            M(index, 1) = pos;  
            if(pos >= FPos(i))
                M(index, 2) = M(index, 2) + mx*(pos - FPos(i));              
            end
        end
    end
end

%{
Pd = Diametral Pitch (teeth/inch)
N = Number of Teeth
n = Angular Velocity (rpm)
F = Face Width (inches)
WtM = Tangetntial load (Newtons)
m = gear Ratio
phi = pressure angle (Rad)
duty = number of revs
%}
function [OUTPUT] = stresses(Pd, N, n, F, WtM, m, phi, duty) 
    dp = N/Pd;             %Diameter inches
    V = pi*dp*n/12;        %Linear Velocity    
    Wt = 0.224809*WtM ;     % Newtons -> lbf
    
    
    %Ko - Overload Factor
    Ko = 1.25; %uniform - MODERATE SHOCK

    %Kv - Dynamic Factor
    Qv = 9; %Low end of precision quality gears, Catalog: pg. 48, AGMA class: 9
    B = 0.25*(12 - Qv)^(2/3);
    A = 50 + 56*(1 - B);
    Kv = ((A + V^0.5)/A)^B;
    
    %Ks - Size Factor  
    x = 1.5708/Pd; %Tooth Width, equation from boston
    Y = 2*x*Pd/3;
    Ks = 1.192*((F*Y^0.5)/Pd)^0.0535;
    
    %Km - Load Distribution Factor
    Cmc = 0.8; %Crowned
    Cpf = F/(10*dp) - 0.025;
    Cpm = 1;
    A = 0.0675; %PRECISION, enclosed, Table 14-9
    B = 0.0167;
    C = -0.926E-4;
    Cma = A + B*F + C*F^2;
    Ce = 1;
    Km = 1 + Cmc*(Cpf*Cpm + Cma*Ce);

    %Kb - Rim Thickness Factor
    Kb = 1; %Assumption given by professor
    %{
    if Pd < 20 %Whole Depth Equation from Boston
        ht = 2.157/Pd;
    else
        ht = 2.2/Pd + 0.002; 
    end
    
    tr = %Cannot find equation or values specifying rim thickness
    
    mb = tr/ht;
    if mb < 1.2
        Kb = 1.6*log(2.242/mb);
    else
        Kb = 1;
    end
    %}
    
    %J - Bending Strength Geometry Factor Fig. 14-6, Very bad lookup table
    %for selected gears
    if N == 16
        J = 0.27;
    else
        J = 0.41;
    end
    
    %Kt - Temperature Factor
    Kt = 1; %Less than 250F
    
    %Kr - Reliability Factor
    Kr = 1; %0.99 - Eq. 14-38
    
    %Cp - Elastic Coefficient
    Cp = 2300; %Table 14-8 steel-steel
    
    %I - Pitting Resistance Geometry Factor
    mn = 1; %Load sharing ratio: 1 for spur gears
    I = ((cos(phi)*sin(phi))/(2*mn))*(m/(m+1));
    
    %Cf - surface condition factor
    Cf = 1;

    %Contact Stress
    CS = (Cp*sqrt(Wt*Ko*Kv*Ks*(Km/(dp*F))*(Cf/I))); %Contact stress in psi

    %Bending Stress
    BS = (Wt*Ko*Kv*Ks*(Pd/F)*((Km*Kb)/J)); %Contact stress in psi 
    
    %Factor of Safety
    
    %Table 14-6, Grade 3 steel, carburized and hardened
    St = 75000;
    Sc = 275000; %psi
    
    if duty > 10^4 %Fig 14-15
        Zn = 2.466*duty^-0.056;
    else
        Zn = 1.5;
    end
        
    Ch = 1; 
    
    if duty > 10^6 %Fig 14-14
        Yn = 1.6831*duty^-0.0323;
    else
        Yn = 6.1514*duty^-0.1192;
    end

    %Contact FOS
    CFOS = ((Sc*Zn*Ch)/(Kt*Kr))/CS;
    
    %Bending FOS
    BFOS = ((St*Yn)/(Kt*Kr))/BS;
    
    CS = CS*0.00689476; %psi -> MPa
    BS = BS*0.00689476;
    
    OUTPUT = [Pd, CS, BS, CFOS, BFOS, Ko, Kv, Ks, Km, Kb, Kt, Kr, Cf, J, I];
end


