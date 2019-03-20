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
k = 1                                                   %Teeth Depth (1 = full)
m1 = 5                                                  %Gear Ratio Stage 1
m2 = 4                                                  %Gear Ratio Stage 2

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
MotorLineE = Efficiency.*MotorLine

f1 = figure('Renderer', 'painters', 'Position', [10 10 1200 300])
subplot(1,3,1)
plot([StallSpeed:NoLoadSpeed], polyval(MotorLine,[StallSpeed:NoLoadSpeed]))
hold on
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

outputPower = reqOutputSpeed*reqOutputTorque*pi/30 
intermediatePower = outputPower/Efficiency^0.5
inputPower = intermediatePower/Efficiency^0.5
efficiency = outputPower/inputPower;

%Shaft C
SpeedC = reqOutputSpeed;
TorqueC = reqOutputTorque;
%Shaft B
SpeedB = SpeedC*m2;
TorqueB = intermediatePower/(SpeedB*pi/30);
%Shaft A
SpeedA = SpeedB*m1;
TorqueA = inputPower/(SpeedA*pi/30);

TorqueSpeedEstimation = table(TorqueA, SpeedA, TorqueB, SpeedB, TorqueC, SpeedC)
PowerTable = table(inputPower, intermediatePower, outputPower, efficiency)

%% Operating Gear Forces, Torques and Diameters


Force1t = [0]
Force1r = [0]
Force2t = [0]
Force2r = [0]

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
    
    %Transmitted Load: Pinion 1 -> Gear 1
    Wt1 = (60000*inputPower*10^-3)/(pi*P1dia*SpeedA);
    %Radial Load: Pinion 1 -> Gear 1
    Wr1 = tan(PressureAngle1)*Wt1;
    
    %Transmitted Load: Pinion 2 -> Gear 2
    Wt2 = (60000*outputPower*10^-3)/(pi*N2dia*SpeedC);
    %Radial Load: Pinion 2 -> Gear 2
    Wr2 = tan(PressureAngle1)*Wt2;
    
    Force1t = [Force1t, Wt1];
    Force1r = [Force1r, Wr1];
    Force2t = [Force2t, Wt2];
    Force2r = [Force2r, Wr2];
end

Force1t = transpose(Force1t(2:end));
Force2t = transpose(Force2t(2:end));
Force1r = transpose(Force1r(2:end));
Force2r = transpose(Force2r(2:end));

P1diameter = transpose(P1diameter(2:end)).*100; %meters -> centimeters;
N1diameter = transpose(N1diameter(2:end)).*100; %meters -> centimeters;
P2diameter = transpose(P2diameter(2:end)).*100; %meters -> centimeters;
N2diameter = transpose(N2diameter(2:end)).*100; %meters -> centimeters;

ForceDiameterTable = table(Force1t, Force1r, P1diameter, N1diameter, Force2t, Force2r, P2diameter, N2diameter)

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

toothHeights = []

ContactStresses = [0];
BendingStresses = [0];

    Pd = 24;
    dp = P1/Pd;     %Pinion Diameter inches
    n = 20000;   %Speed rpm
    F = 0.25;       %Facewidth
    V = pi*dp*n/12; %Linear Velocity
    Wt = 500;           %Tangential load
    
    
    %Ko - Overload Factor
    Ko = 1.2; %Moderate Shock

    %Kv - Dynamic Factor
    Qv = 7 %High end of commercial quality gears
    B = 0.25*(12 - Qv)^(2/3)
    A = 50 + 56*(1 - B)
    Kv = ((A + V^0.5)/A)^B
    
    %Ks - Size Factor  
    x = 0.05; %Tooth Width
    Y = 2*x*Pd/3;
    Ks = 1.192*((F*Y^0.5)/Pd)^0.0535;
    
    %Km - Load Distribution Factor
    Cmc = 0.8; %Crowned
    Cpf = F/(10*dp) - 0.025;
    Cpm = 1;
    A = 0.127; %Commercial, enclosed
    B = 0.0158;
    C = -0.930E-4;
    Cma = A + B*F + C*F^2;
    Ce = 1;
    Km = 1 + Cmc*(Cpf*Cpm + Cma*Ce);

    %Kb - Rim Thickness Factor
    %{
    tr %Rim thickness
    ht =  %Tooth Height    
    mb = tr/ht;
    if mb < 1.2
        Kb = 1.6*log(2.242/mb)
    else
        Kb = 1
    end
    %}
    Kb = 1
    
    %J - Bending Strength Geometry Factor
    J = 0.275 %Figure 14-6
    
    %St
    St = 1;
    
    %Yn - Stress Cycle Factor
    Yn = 1;
    
    %Kt - Temperature Factor
    Kt = 1;
    
    %Kr - Reliability Factor
    Kr = 1; %0.99 - Eq. 14-38
    
    %Cp - Elastic Coefficient
    Cp = 2300; %Table 14-8
    
    %I - Pitting Resistance Geometry Factor
    mg = m1;
    phi = PressureAngle1;
    mn = 1; %Load sharing ratio: 1 for spur gears
    I = ((cos(phi)*sin(phi))/(2*mn))*(mg/(mg+1));
    
    %Cf - surface condition factor
    Cf = 1
    
    %Sc - allowable Constact Stress
    Sc = 275000 %psi  Allowable constact stress at 10^7 cycles and 99% reliability
    
    %Zn - Stress-Cycle Factor
    Zn = 1
    
    %Ch - hardness ratio factor
    Hbp = 627 %Brinell hardness of pinion
    Hbg = 627 %Briness hardness of gear
    Hb = Hbp/Hbg;
    Ch = Hb;
    
    
    %Contact Stress
    CS = Cp*(Wt*Ko*Kv*Ks*(Km/(dp*F))*(Cf/I))^0.5 %Contact stress in psi
    %ContactStresses = [ContactStresses, CS];

    %Bending Stress
    BS = Wt*Ko*Kv*Ks*(Pd/F)*((Km*Kb)/J)
    %BendingStresses = [BendingStresses, BS];


ContactStresses = ContactStresses(2:end);
ContactStresses = transpose(ContactStresses).*0.00689476; %psi -> MPa
BendingStresses = BendingStresses(2:end);
BendingStresses = transpose(BendingStresses).*0.00689476; %psi -> MPa
