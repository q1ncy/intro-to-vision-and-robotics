% Chapter 2 
clc; clear; close all;
%% when executing this perform it block by block as some function are
%%defined mutiple times 
%proceed by uncommenting the desired part and running it 
%to make it easy, i have seperated diffrenet sections by bold comments 
%the labels are also in line with the submitted pdf document 
%% 1.1 2d rotation matrix 
    % R =  rot2(0.3);
    % trplot2(R);
    % disp('R =');
    % disp(R);
  
%% 1.2 Pose in 2d 
   %computing two relative poses 
   % TA = trvec2tform([1 2])*tformr2d(deg2rad(30));
   % axis([0 5 0 5]);
   % plottform2d(TA,frame="A",color="b");
   % grid on;
   % hold on;
   % %adding our reference frame
   % T0 = trvec2tform([0 0]);
   % plottform2d(T0, frame="0",color="k");
   % %proof of non commutativity of transformation
   % TB = trvec2tform([2 1]);
   % plottform2d(TB,frame="B",color="r");
   % TAB = TA*TB;
   % plottform2d(TAB,frame="AB",color="g");
   % TBA = TB*TA;
   % plottform2d(TBA,frame="BA",color="c");
   % %defining point P
   % P = [3;2];
   % plotpoint(P',"ko",label="P");
   % %determining homogenous coordinate of P wrt A
   % P_A = homtrans(inv(TA),P');
%% 1.3 Rotating a coordinate frame
% axis([-5 4 -1 5]);
% T0 = trvec2tform([0 0]);%reference frame
% 
% plottform2d(T0,frame="0",color="k");
% grid on;
% TX = trvec2tform([2 3]); %target frame
% plottform2d(TX,frame="X",color="b");
% %create a 2d rotational matrix of angle 2 radians
%TR = tformr2d(2);
% 
% plottform2d(TR*TX,frame="RX",color="g");
% plottform2d(TX*TR,framelabel="XR",color="g");
% % adding a point C 
%C = [3,2];
% plotpoint(C,"ko",label="C");
% % formation of a transformation matrix about the z axis
%TC = trvec2tform(C)*TR*trvec2tform(-C);
% % applying TC to target frame TX
% plottform2d(TC*TX,frame="XC",color="r");

%% 1.4 Matrix exponential for pose
% L = logm(TC);
% S = skewa2vec(L);
% disp('Our rotational matrix:');
% disp(L);
% disp('Our skew-symmetric matrix:')
% disp(S);
% Og = logm(expm(vec2skewa (S)));
% disp('getting our original rotational matrix;')
% disp(Og);
%% 1.5 2D twists 
%creating a twist about point C
% S= Twist2d.UnitRevolute(C);
% %using the twist to obtain a rotation of 2 rads
% R=  expm(vec2skewa(2*S.compact));
% %alternatively we can use 
% R_a = S.tform(2); % this is equal to the log form of TC
% %we can get the pole using 
% Pol= S.pole;
% disp('Our twist is represented as:');
% disp(S);
% disp('From this we can obtain the rotaional matrix as:');
% disp(R);
% disp('Whose pole is given by:')
% disp(Pol);
    %% translational motion in y-direction 
% S = Twist2d.UnitPrismatic([0 1]); %displacement of 1 
% R=S.tform(2);
% disp("The translation twist is given as:");
% disp(S);
% disp("The result is a zero rotational matrix in z axis");
% disp("But a translation of 2 in y-axis as shown:");
% disp(R);
    %% For an arbitrary 2D homogenous transformation
% T = trvec2tform([3 4])*tformr2d(0.5);
% S = Twist2d(T);
% Angle= S.w; %in radians
% Pol=S.pole;
% Og = S.tform();
% disp('Our homogenous transformation matrix is:');
% disp(T);
% disp('and the twist is');
% disp(S);
% disp('The rotational angle which is the first bit is:')
% disp(Angle);
% disp("Rotated about the point:");
% disp(Pol);
% disp('using .tform() to exponent our twist we obtain our original as:')
% disp(Og);
%% 1.6 3D rotational Matrix 
%we can use inbuilt function to genrate rotational matrices
% R = rotmx(pi/2);
% plottform(R);
% grid on;
% animtform(R);
% disp('the 3d rotation matrix in about the x axis by 90 deg is;')
% disp(R);
% disp('obtained using the function rotmx()')
% applying another rotation about y-axis
% Rxy = rotmx(pi/2)*rotmy(pi/2);
% Ryx = rotmy(pi/2)*rotmx(pi/2);
% plottform(Rxy);
% grid on;
% animtform(Rxy);
% disp('A rotation about x then y is given by:')
% disp(Rxy);
%comment of the prior before executing the next
% plottform(Ryx);
% grid on;
% animtform(Ryx);
% disp('A rotation about y then x is given by:')
% disp(Ryx);
%% 1.7 Three- Angle representations 
% %in mechanical dynamics, ZYZ sequencey is commonly used 
% R = eul2rotm([0.1 -0.2 0.3],"ZYZ");
% %we can find the euler angles to this by
% gamma = rotm2eul(R,"ZYZ");
% %wher the angle is positive 
% Rp = eul2rotm([0.1 0.2 0.3],"ZYZ");
% gammap = rotm2eul(Rp,"ZYZ");
% %we can obtain the original rotation matrix by 
% Og =  eul2rotm(gamma,"ZYZ");
% % for an angle of 0 deg 
% Ro = eul2rotm([0.1 0 0.3],"ZYZ");
% gammao = rotm2eul(Ro,"ZYZ");
% disp('The Z Y Z roational matrix of +0.2 rads is :');
% disp(R);
% disp('From this we can obtain the euler angles as:');
% disp(gamma);
% disp('Where the angle is positive the rotational matrix is;');
% disp(Rp);
% disp("and the euler angles are:")
% disp(gammap);
% disp('as seen computing the inverse returns a different set of ZYZ euler angles');
% disp('however getting the inverse of the euler angles give us the same matrix');
% disp(Og);
% disp("for an angle of 0 deg");
% disp(Ro);
% disp("And the inverse of this gives us the following euler angels:")
% disp(gammao);
%% 1.8 Unit Quaternion
%construction 
% q = quaternion(rotmx(0.3),"rotmat","point");
% mult = q*q;
% %to obtain conjugate
% con = q.conj;
% %inverse multiplication
% I = q*con;
% %converting to rotation matrix
% R= q.rotmat("point");
% %obtaining its vector components 
% V= q.compact();
% %rotation of coordinate point
% Rp= q.rotatepoint([0 1 0]);
% disp('A quaternion can be constructed as:');
% disp(q);
% disp('If we perform a quaternion multiplication of itself we get:');
% disp(mult);
% disp('We can obtain the inverse of the original quaternion by:');
% disp(con);
% disp('multiplyig the inverse by the original we get');
% disp(I);
% disp('which is an identity quaternion');
% disp('When we convert a quaternion to a rotation matrix we get:');
% disp(R);
% disp('And we obtain its vector components as:');
% disp(V);
% disp('We can also use a quaterion to rotate a coordinate point')
% disp('In this instant we rotate the point [0 1 0] to get:');
% disp(Rp);
%% 1.9 3D twists 
% %rotation about x axis
% %generating a unit twist rotating by 1 rad about x
% S = Twist.UnitRevolute([1 0 0],[0 0 0]);
% T = S.tform(0.3);
% %illustrating screw motion 
% Sm = Twist.UnitRevolute([0 0 1],[2 3 2],0.5);
% %coordinate frame
% X = trvec2tform([3 4 -4]);
% clf;
% hold on
% view(3)
% for theta = [0:0.3:15]
%     plottform(Sm.tform(theta)*X,style="rgb",LineWidth=2)
%     grid on
% end
% L = Sm.line();
% L.plot("k:",LineWidth=2);
% disp('we obtain a unit twist rotation of 1 rad about x as:')
% disp(S)
% disp('Its skew symmetrix matrix is given by:')
% disp(T);


