
%initializing the tetha vectors for easier processing:

Tetha_vec1=zeros(1, 101);
Tetha_vec2=zeros(1, 101);
Tetha_vec3=zeros(1, 101);
Tetha_vec4=zeros(1, 101);
Tetha_vec5=zeros(1, 101);

% Defining the variables radius
a = 225;
b = 40;
%Setting the center of the ellipse
center = [225,0,400]; 
%vector of 
points = ones(101,3);

%Setting the number of loops
for k =1:101    

%Defining the equation of the ellipse
fi = (2*pi/100)*(k-1);
x = center(1) + a * cos(fi); 
y = center(2) + b * sin(fi); 
z = center(3);

% Store the points in a matrix להחליףףףףףףף
points(k,:) = [x , y, z]*rotx(-32)*roty(23)*rotz(20);
end
%ellipse angles solution:

for j=1:101


[T1,T2,T3,T4,T5] = ik(points(j,:));

%filling the vectors with the solutions:
Tetha_vec1(j)=Tetha_vec1(j)+T1;
Tetha_vec2(j)=Tetha_vec2(j)+T2;
Tetha_vec3(j)=Tetha_vec3(j)+T3;
Tetha_vec4(j)=Tetha_vec4(j)+T4;
Tetha_vec5(j)=Tetha_vec5(j)+T5;

end
%solver function:
function [Tetha1,Tetha2,Tetha3,Tetha4,Tetha5]= ik(xyz)

theta1 = sym('tetha1');
theta2 = sym('tetha2');
theta3 = sym('tetha3');
theta4 = sym('tetha4');
theta5 = sym('tetha5');

HTM=eye(4);
DhMat = [ 0    0    107 theta1;
          20 90   0   theta2;
          150   0    0   theta3;
          100   0    0   theta4;
          68.5   0    0   theta5;
          29  -90   239   0];

for i = 1:6
    (i); TM_iim1 = DH(DhMat(i,1),DhMat(i,2),DhMat(i,3),DhMat(i,4));
    (i); HTM=HTM*TM_iim1;
end

HTM60 = HTM;

nx = HTM60(1,2);
ny = HTM60(2,2);
nz = HTM60(3,2);
Pxyz = [HTM60(1,4), HTM60(2,4), HTM60(3,4)];
Tn = [0, 1, 0]*rotx(20);
Tar = xyz;

var_fun = @(vars) [double(subs(Tn(1)-nx, [theta1, theta2, theta3, theta4, theta5], vars));
                             double(subs(Tn(2)-ny, [theta1, theta2, theta3, theta4, theta5], vars));
                             double(subs(Tn(3)-nz, [theta1, theta2, theta3, theta4, theta5], vars));
                             double(subs(Pxyz(1)-Tar(1), [theta1, theta2, theta3, theta4, theta5], vars));
                             double(subs(Pxyz(2)-Tar(2), [theta1, theta2, theta3, theta4, theta5], vars));
                             double(subs(Pxyz(3)-Tar(3), [theta1, theta2, theta3, theta4, theta5], vars))];

initial_Guess = [0, 0, 0, 0, 0];

Options = optimoptions('lsqnonlin', 'Display', 'iter');
Solution = lsqnonlin(var_fun, initial_Guess, [-360,-180,-45,-10,-10], [360,180,100,28,10], Options);


Tetha1 = Solution(1);
Tetha2 = Solution(2);
Tetha3 = Solution(3);
Tetha4 = Solution(4);
Tetha5 = Solution(5);

end
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

