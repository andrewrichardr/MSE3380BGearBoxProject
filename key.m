%Key calcs

%converts shaft torque into lbf.in
shaft1 = 3.19*8.85;
shaft2 = 13.25*8.85;
shaft3 = 55*8.85;

Sy = 57000;%Based on 1020 material props.
GL = 0.5; %length of the gears

%Bore Diameter This is diameter of the shaft for the gears
g1 = 0.5;
g2 = 0.375;

G1 = 0.09375;%Based on tables in the book converts diameter into key dimensions
G2 = 0.125;

%Gear 1
F1 = shaft1/(g2/2);
FOS = Sy/(F1/(G1*GL/2));

%Gear 2
F2 = shaft2/(g1/2);
FOS2 = Sy/(F2/(G2*GL/2));

%Gear 3 
F3 = shaft2/(g2/2);
FOS3 = Sy/(F3/(G1*GL/2));

%Gear 4
F4 = shaft3/(g1/2);
FOS4 = Sy/(F4/(G2*GL/2));

