%Bearings Calculations

InputS = 11005.86;
InputT = 3.19;
Reduction1 = 4.375;
Reduction2 = Reduction1;
R1 = 44.45;
R2 = 10.15;
R3 = R1;
R4 = R2;
PA = deg2rad(20);
a = 3; % ball bearing constant
Life = 20000;


Db = 24.89; %Motor to Gear
Da = 2; %Gear to Outer Bearing
Dc = 13.00; % C to pinion
Dd = 2.00; %D to gear
De = 30.48; %Gap between gears
Df = 2;%Gear to bearing    
Dg = 7.62; % bearing to bearing

%Input Shaft
Ft = 0.31425;% from gear resultant calc 
Fr = Ft*tan(PA);
Fres = Ft/cos(PA);
Fa = Fres*(Db/(Db+Da))

%Intermediate Shaft
Ft2 = 0.29812; %from gear resultant calc
Fr2 = Ft2*tan(PA)
Fres2 = sqrt(Ft2^2+Fr2^2);

Ft3 = 1.304; % from gear resultant calc
Fr3 = Ft3*tan(PA)
Fres3 = sqrt(Ft3^2+Fr3^2);

Fct = (Ft3*(De+Dd)+Ft2*Dd)/(Dd+De+Dc);
Fcr = (Fr3*(De+Dd)-Fr2*Dd)/(Dd+De+Dc);
Fc = sqrt(Fct^2+Fcr^2)

Fdt = (Ft3*(Dc)+Ft2*(De+Dc))/(Dd+De+Dc);
Fdr = (Fr3*(Dc)-Fr2*(De+Dc))/(Dd+De+Dc);
Fd = sqrt(Fdt^2+Fdr^2)

%Output Shaft 

Ft4 = 1.23725;%from gear resultant calc
Fr4 = 0.45;
Fres4 = sqrt(Ft4^2+Fr4^2);

Fe = Fres4/(Df+Dg)*Dg;
Ff = Fres4*(Df/(Df+Dg));

%Bearing Calculations

Xda = 60*Life*InputS/(10^6);
C10a = Fa*(power(Xda,(1/a)))

Xdc = (60*Life*InputS/Reduction1)/(10^6);
C10c = Fc*(power(Xdc,(1/a)))

Xdd = (60*Life*InputS/Reduction1)/(10^6);
C10d = Fd*(power(Xdd,(1/a)))

Xde = (60*Life*InputS/Reduction1^2)/(10^6);
C10e = Fe*(power(Xde,(1/a)))

Xdf = (60*Life*InputS/Reduction1^2)/(10^6);
C10f = Ff*(power(Xdf,(1/a)))