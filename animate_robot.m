% animate_robot.m - animates the 5-DOF arm along the ellipse.
% Run ik_ellipse.m first: it creates Tetha_vec1..5 and points in the workspace.

syms theta1 theta2 theta3 theta4 theta5 real

theta1 = Tetha_vec1;
theta2 = Tetha_vec2;
theta3 = Tetha_vec3;
theta4 = Tetha_vec4;
theta5 = Tetha_vec5;
%Bas
n0=stlread('models/Base.STL');
n0_=n0.ConnectivityList;
XYZ0=n0.Points;
XYZ0(:,1)= XYZ0(:,1)-50;
XYZ0(:,2)= XYZ0(:,2)-50;
XYZ0(:,3)=XYZ0(:,3);

for i=1:100
%Flach
n1=stlread('models/Flach.STL');
n1_=n1.ConnectivityList;
XYZ1=n1.Points;
XYZ1(:,1)= XYZ1(:,1)-25;
XYZ1(:,2)= XYZ1(:,2)-25;
XYZ1(:,3)=XYZ1(:,3)-26;
XYZ1=XYZ1*rotz(90);
T01=DH(0, 0, 107, theta1(i));
ones1=ones(1,size(XYZ1,1));
XYZ1_n=T01*[XYZ1'; ones1];
XYZ1_n=(XYZ1_n(1:3,:))';

%link 150
n2=stlread('models/Link 150.STL');
n2_=n2.ConnectivityList;
XYZ2=n2.Points;
XYZ2(:,1)= XYZ2(:,1)-10;
XYZ2(:,2)= XYZ2(:,2)-10;
XYZ2(:,3)=XYZ2(:,3)-23;
T12=DH(20, 90, 0, theta2(i));
T02 = T01*T12;
ones2=ones(1,size(XYZ2,1));
XYZ2_n=T02*[XYZ2'; ones2];
XYZ2_n=(XYZ2_n(1:3,:))';

% %link 100
n3=stlread('models/Link 100.STL');
n3_=n3.ConnectivityList;
XYZ3=n3.Points;
XYZ3(:,1)= XYZ3(:,1)-10;
XYZ3(:,2)= XYZ3(:,2)-10;
XYZ3(:,3)= XYZ3(:,3)-21;
T23=DH(150, 0, 0, theta3(i));
T03 = T01*T12*T23;
ones3=ones(1,size(XYZ3,1));
XYZ3_n=T03*[XYZ3'; ones3];
XYZ3_n=(XYZ3_n(1:3,:))';


% Cube 
n4=stlread('models/Cube.STL');
n4_=n4.ConnectivityList;
XYZ4=n4.Points;
XYZ4(:,1)= XYZ4(:,1)-10;
XYZ4(:,2)= XYZ4(:,2)-15;
XYZ4(:,3)= XYZ4(:,3)-30;
T34=DH(100, 0, 0, theta4(i));
T04 = T01*T12*T23*T34;
ones4=ones(1,size(XYZ4,1));
XYZ4_n=T04*[XYZ4'; ones4];
XYZ4_n=(XYZ4_n(1:3,:))';

%Hold
n5=stlread('models/Hold.STL');
n5_=n5.ConnectivityList;
XYZ5=n5.Points;
XYZ5=XYZ5*roty(90);
XYZ5(:,1)= XYZ5(:,1)+3;
XYZ5(:,2)= XYZ5(:,2)-6;
XYZ5(:,3)= XYZ5(:,3)-6;
XYZ5=XYZ5*rotz(180);
T45=DH(68.5, 0, 0, theta5(i));
T05 = T01*T12*T23*T34*T45;
ones5=ones(1,size(XYZ5,1));
XYZ5_n=T05*[XYZ5'; ones5];
XYZ5_n=(XYZ5_n(1:3,:))';

%Pencil
n6=stlread('models/Pencil.STL');
n6_=n6.ConnectivityList;
XYZ6=n6.Points;
XYZ6=XYZ6*rotx(90);
XYZ6(:,1)= XYZ6(:,1)-5.5;
XYZ6(:,2)= XYZ6(:,2)-5.9;
XYZ6(:,3)= XYZ6(:,3);
T56=DH(29, -90, 239, 0);
T06 = T01*T12*T23*T34*T45*T56;
ones6=ones(1,size(XYZ6,1));
XYZ6_n=T06*[XYZ6'; ones6];
XYZ6_n=(XYZ6_n(1:3,:))';


n0=trisurf(n0_,XYZ0(:,1),XYZ0(:,2),XYZ0(:,3),'Facecolor', [0 0 1], 'EdgeColor' , 'k');
hold on;
n1=trisurf(n1_,XYZ1_n(:,1),XYZ1_n(:,2),XYZ1_n(:,3),'Facecolor', [1 0 0], 'EdgeColor' , 'k');
n2=trisurf(n2_,XYZ2_n(:,1),XYZ2_n(:,2),XYZ2_n(:,3),'Facecolor', [1 1 0], 'EdgeColor' , 'k');
n3=trisurf(n3_,XYZ3_n(:,1),XYZ3_n(:,2),XYZ3_n(:,3),'Facecolor', [0 1 0], 'EdgeColor' , 'k');
n4=trisurf(n4_,XYZ4_n(:,1),XYZ4_n(:,2),XYZ4_n(:,3),'Facecolor', [0 1 1], 'EdgeColor' , 'k');
n5=trisurf(n5_,XYZ5_n(:,1),XYZ5_n(:,2),XYZ5_n(:,3),'Facecolor', [0 0 1], 'EdgeColor' , 'k');
n6=trisurf(n6_,XYZ6_n(:,1),XYZ6_n(:,2),XYZ6_n(:,3),'Facecolor', [1 0 1], 'EdgeColor' , 'k');
el=plot3(points(:,1), points(:,2), points(:,3), '.r');
hold off;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-700 700]);
ylim([-700 700]);
zlim([-700 700]);

pause (0.01);
 end
T06
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