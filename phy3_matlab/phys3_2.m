gravitationalConstant = 6.67408e-11;
sunMass = 1.98855e30;
orbitalPeriodEarth = 365.24 * 24 * 60 * 60;
orbitalPeriod = 2400 * orbitalPeriodEarth;
auInMeters =  149597870700;
closestDistance = auInMeters;
meanDistanceToSunEarth = 149.6 * 1e9;
elipseMajorAxisLengthEarth = meanDistanceToSunEarth;


% orbitalPeriod^2 / orbitalPeriodEarth^2 = elipseMajorAxisLength^3 / elipseMajorAxisLengthEarth^3 
elipseMajorAxisLength = nthroot(orbitalPeriod^2 / orbitalPeriodEarth^2 * elipseMajorAxisLengthEarth^3,3);
meanDistanceToSun = elipseMajorAxisLength;
disp("meanDistanceToSun: " + meanDistanceToSun);
disp("meanDistanceToSun in AU: " + meanDistanceToSun / auInMeters);

maxDistance = 2 * elipseMajorAxisLength - closestDistance;
disp("maxDistance: " + maxDistance);
disp("maxDistance in AU: " + maxDistance / auInMeters);

% speedClose * closestDistance = speedMaxDistance * maxDistanceToSun
% speedClose / speedMaxDistance = maxDistanceToSun / closestDistance
speedRatio = maxDistance / closestDistance;
disp("Speed Ratio: " + speedRatio);