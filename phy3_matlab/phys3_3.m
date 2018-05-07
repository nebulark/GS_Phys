g = 9.81;
pianoMass = 380;
incline = deg2rad(25);
displacement = 2.9;
forewardsForce = pianoMass*g*sin(incline);
disp("Mans force = " + (-forewardsForce));

disp("Work done on Piano by men = " + (-forewardsForce * displacement));
disp("Work done on Piano by gravity = " + (forewardsForce * displacement));
