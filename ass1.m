% Defining the Poses (x, y, theta_degrees) 
% Pose 1: (1, 2, 30 degrees)
theta1 = 30; 
x1 = 1;
y1 = 2;

% Pose 2: (2, 1, 0 degrees)
theta2 = 0;
x2 = 2;
y2 = 1;

% T1 Matrix
T1 = [cosd(theta1), -sind(theta1), x1;
      sind(theta1),  cosd(theta1), y1;
      0,             0,            1];

% T2 Matrix
T2 = [cosd(theta2), -sind(theta2), x2;
      sind(theta2),  cosd(theta2), y2;
      0,             0,            1];

%solving for problem 
T3 = T1 * T2;
T4 = T2 * T1;

% -Displaying Results
disp('Solution for T3=(T1 * T2):');
disp(T3);
disp('Solution for  T$=(T2 * T1):');
disp(T4);


