clear
clc

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
FaceWidth = [1.5, 1.25, 1, 0.75, 0.5, 0.25, 0.188, 0.125]         %inch
DiametralPitch = [8, 10, 12, 16, 20, 24, 32, 48]                    %teeth/inch
DiametralPitchChosen = 20;
k = 1                                                               %Teeth Depth (1 = full)
%Number of teeth Chosen from iterative analysis, 
%Chosen diametral pich of 20
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
MotorLine = polyfit(points(:,1),points(:,2),polynomialDegree);
MotorLineE = Efficiency.*MotorLine;

f1 = figure('Renderer', 'painters', 'Position', [10 10 1200 300])
subplot(1,3,1)
plot([StallSpeed:NoLoadSpeed], polyval(MotorLine,[StallSpeed:NoLoadSpeed]))
hold on
title("Motor Torque-Speed")
xlabel("Motor Speed (rpm)")
ylabel("Motor Torque (N.m)")
hold off

%% Gear Ratio Target (minimization)
syms r
eqn = MotorLineE(1)*reqOutputSpeed*r^2 + MotorLineE(2)*r == reqOutputTorque;
soln = double(solve(eqn,r))

ratioTarget = soln(1);

TargetLine = [MotorLineE(1)*ratioTarget^2, MotorLineE(2)*ratioTarget];

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

RealLine = [MotorLineE(1)*ratio^2, MotorLineE(2)*ratio];

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

TTorqueSpeedEstimation = table(TorqueA, SpeedA, TorqueB, SpeedB, TorqueC, SpeedC)
TPower = table(inputPower, intermediatePower, outputPower, efficiency)

%% Operating Gear Forces, Torques and Diameters

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
    
    Force1t = [Force1t, Wt1*1000]; %kN -> N
    Force1r = [Force1r, Wr1*1000];
    Force2t = [Force2t, Wt2*1000];
    Force2r = [Force2r, Wr2*1000];
end

Force1t = transpose(Force1t(2:end));
Force2t = transpose(Force2t(2:end));
Force1r = transpose(Force1r(2:end));
Force2r = transpose(Force2r(2:end));

P1diameter = transpose(P1diameter(2:end)).*100; %meters -> centimeters;
N1diameter = transpose(N1diameter(2:end)).*100; %meters -> centimeters;
P2diameter = transpose(P2diameter(2:end)).*100; %meters -> centimeters;
N2diameter = transpose(N2diameter(2:end)).*100; %meters -> centimeters;

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
 
    Pinion1 = [Pinion1; stresses(DiametralPitch(x), P1, SpeedA, FaceWidth(x), Force1t(x), m1actual, PressureAngle)];
    Gear1 = [Gear1; stresses(DiametralPitch(x), N1, SpeedB, FaceWidth(x), Force1t(x), m1actual, PressureAngle)];
    Pinion2 = [Pinion2; stresses(DiametralPitch(x), P2, SpeedB, FaceWidth(x), Force2t(x), m2actual, PressureAngle)];
    Gear2 = [Gear2; stresses(DiametralPitch(x), N2, SpeedC, FaceWidth(x), Force2t(x), m2actual, PressureAngle)];

    if DiametralPitch(x) == DiametralPitchChosen
       PINION1 =  stresses(DiametralPitch(x), P1, SpeedA, FaceWidth(x), Force1t(x), m1actual, PressureAngle);
       GEAR1 =  stresses(DiametralPitch(x), N1, SpeedB, FaceWidth(x), Force1t(x), m1actual, PressureAngle);
       PINION2 =  stresses(DiametralPitch(x), P2, SpeedB, FaceWidth(x), Force2t(x), m2actual, PressureAngle);
       GEAR2 =  stresses(DiametralPitch(x), N2, SpeedC, FaceWidth(x), Force2t(x), m2actual, PressureAngle);
    end
    
end

Pinion1 = Pinion1(2:end,:);
Gear1 = Gear1(2:end,:);
Pinion2 = Pinion2(2:end,:);
Gear2 = Gear2(2:end,:);

names = ["DP","ContactStress", "BendingStress", "ContactFOS", "BendingFOS", "Ko", "Kv", "Ks", "Km", "Kb", "Kt", "Kr", "Cf", "J", "I"];
Tpinion1 = array2table(Pinion1, 'VariableNames', names)
Tgear1 = array2table(Gear1, 'VariableNames', names)
Tpinion2 = array2table(Pinion2, 'VariableNames', names)
Tgear2 = array2table(Gear2, 'VariableNames', names)

PINION1 = transpose(PINION1);
GEAR1 = transpose(GEAR1);
PINION2 = transpose(PINION2);
GEAR2 = transpose(GEAR2);

Tselected = table(transpose(names), PINION1, GEAR1, PINION2, GEAR2)

%{
Pd = Diametral Pitch (teeth/inch)
N = Number of Teeth
n = Angular Velocity (rpm)
F = Face Width (inches)
WtM = Tangetntial load (Newtons)
m = gear Ratio
phi = pressure angle (Rad)
%}
function [OUTPUT] = stresses(Pd, N, n, F, WtM, m, phi) 
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
    Kt = 1;
    
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
    
    %Table 14-6, Grade 2 steel, carburized and hardened
    St = 65000;
    Sc = 225000; %psi
    
    Zn = 1; %Fig 14-15
    Ch = 1; 
    Yn = 1;

    %Contact FOS
    CFOS = ((Sc*Zn*Ch)/(Kt*Kr))/CS;
    
    %Bending FOS
    BFOS = ((St*Yn)/(Kt*Kr))/BS;
    
    CS = CS*0.00689476; %psi -> MPa
    BS = BS*0.00689476;
    
    OUTPUT = [Pd, CS, BS, CFOS, BFOS, Ko, Kv, Ks, Km, Kb, Kt, Kr, Cf, J, I];
end


