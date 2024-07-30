clc
clearvars
close all 

% %Establecer la matriz p
% syms p1x p1y p2x p2y p3x p3y p4x p4y
% 
% pxy=[p1x p1y;
%      p2x  p2y;
%      p3x  p3y];
% 
% %producto kronecker
% I=eye(3);
% kronp=kron(I,pxy);
% 
% %Matrices de incidencia locales cosa rigida
% E1=[1  1  0;
%    -1  0  0;
%     0 -1  0 ];
% E2=[-1  0  0;
%      1  0  1;
%      0  0 -1 ];
% E3=[0  0  -1;
%     0 -1   0;
%     0  1   1 ];
% 
% E=[E1' E2' E3'];
% 
% %Calculo del vector R
% R=E*kronp;


%Establecer la matriz p
syms p1x p1y p2x p2y p3x p3y 

p1=[p1x;p1y];
p2=[p2x;p2y];
p3=[p3x;p3y];

%producto kronecker
I=eye(2);
% kronp=kron(I,pxy);

%Definir las p
e1=p2-p1;
e2=p3-p2;
e3=p1-p3;
e=blkdiag([e1(1) e1(2)],[e2(1) e2(2)],[e3(1) e3(2)]);
%Matrices de incidencia locales cosa rigida
E=[-1  1  0;
    0 -1  1;
    1  0 -1];
kronp=kron(E,I)
%Calculo del vector R
R=e*kronp;

