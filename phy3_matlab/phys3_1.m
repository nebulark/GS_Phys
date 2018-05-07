gravitationalConstant = 6.67408e-11;
massOfMoon = 7.34767309e22;
r = 95e3 + 1740e3;
moonG = gravitationalConstant * massOfMoon

orbitalVelocity = sqrt(gravitationalConstant * massOfMoon / r)

periodTime = (2 * r * pi) / orbitalVelocity;
disp("time for one period is " + periodTime + "Seconds");



