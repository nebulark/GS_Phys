mass = 62;
v0 = 4.5;
g = -9.81;
s0 = 2;
k = 5.8e4;

s = @ (t) (s0 + t * v0  + 0.5 * t^2 * g);
y = 0:0.01:2;
x2 = arrayfun(s,y);
plot (y,x2);
grid on

% calc impact timepoint
% 0 = s0 + t * v0 + 0.5 * t^2 * g
syms t;

value = double(solve(0 == s0 + t * v0 + 0.5 * t^2 * g,t));
timeOfImpact = value(2) % first one is negative


vImpact = v0 + g * timeOfImpact;
impactSpeed = abs(vImpact);
disp("Impact Velocity " + impactSpeed);

% calc timeOfmaxHeight
% v(t) = v0 + t * g
% 0 =  v0 + t * g
% t = -v0 / g
timeOfmaxHeight = -v0 / g
maxHeight = s0 + timeOfmaxHeight * v0  + 0.5 * timeOfmaxHeight^2 * g
potentialEnergy = abs(mass * g * maxHeight)


%federPot = 0.5 * k * x^2

%federPot * 2 / k = x^2

x = sqrt(potentialEnergy * 2 / k);
disp("the trampoline gets depressed by " + x + " meters" );
