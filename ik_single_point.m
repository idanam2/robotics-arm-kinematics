syms theta1 theta2 theta3 theta4 theta5 real;
HTM=eye(4);
DhMat = [ 0    0    107 theta1;
          20 90   0   theta2;
          150   0    0   theta3;
          100   0    0   theta4;
          68.5   0    0   theta5;
          29  -90   239   0];
for i = 1:6
    (i);TM_iim1 = DH(DhMat(i,1),DhMat(i,2),DhMat(i,3),DhMat(i,4));
    (i);HTM=HTM*TM_iim1;
end
HTM60 = HTM; 
nx = HTM60(1,2);
ny = HTM60(2,2);
nz = HTM60(3,2);
Pxyz = [HTM60(1,4), HTM60(2,4), HTM60(3,4)];
n = [0, 1, 0]*rotx(45); %העיפרון בכיוון ציר Y וסיבוב רנדומלי
P = [65,20, 380];
relative_function = @(vars) [double(subs(n(1)-nx, [theta1, theta2, theta3, theta4, theta5], vars)); 
                             double(subs(n(2)-ny, [theta1, theta2, theta3, theta4, theta5], vars));
                             double(subs(n(3)-nz, [theta1, theta2, theta3, theta4, theta5], vars));
                             double(subs(Pxyz(1)-P(1), [theta1, theta2, theta3, theta4, theta5], vars));
                             double(subs(Pxyz(2)-P(2), [theta1, theta2, theta3, theta4, theta5], vars));
                             double(subs(Pxyz(3)-P(3), [theta1, theta2, theta3, theta4, theta5], vars))];
initial_Guess = [0, 0, 0, 0, 0];
 Options = optimoptions('lsqnonlin', 'Display', 'iter'); 
Solution = lsqnonlin(relative_function, initial_Guess, [], [], Options); 
disp('Solution:');
disp(['theta1 =', num2str(Solution(1))]);
disp(['theta2 =', num2str(Solution(2))]);
disp(['theta3 =', num2str(Solution(3))]);
disp(['theta4 =', num2str(Solution(4))]);
disp(['theta5 =', num2str(Solution(5))]);
function Tiimin1=DH(a,alph,d,theta)
ct = cosd(theta);
st = sind(theta);
ca = cosd(alph);
sa = sind(alph);
Rotz = [ct -st 0;
         st ct 0;
          0 0 1];
Pz = [0 0 d]';
Tim = [Rotz Pz; 0 0 0 1];
Rotx = [1 0 0;
        0 ca -sa;
        0 sa ca];
Px = [a 0 0]';
Tmimin1 = [Rotx Px; 0 0 0 1];
Tiimin1= Tmimin1*Tim;
end