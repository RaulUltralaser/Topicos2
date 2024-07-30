clc
clearvars
close all 

%Establecer la matriz p
syms p1x p1y p2x p2y p3x p3y p4x p4y

% p1=[p1x;p1y];
% p2=[p2x;p2y];
% p3=[p3x;p3y];
% p4=[p4x;p4y];

p1=[0;0];
p2=[1;0];
p3=[1;-1];
p4=[0;-1];

%producto kronecker
I=eye(2);

%% Estructura rigida
%Definir las p
e1=p2-p1;
e2=p4-p1;
e3=p3-p2;
e4=p3-p4;
e5=p3-p1;
e6=p4-p2;
e=blkdiag([e1(1) e1(2)],[e2(1) e2(2)],[e3(1) e3(2)],[e4(1) e4(2)],[e5(1) e5(2)],[e6(1) e6(2)]);
%Matrices de incidencia locales cosa rigida
E = [1 -1  0  0;
         1  0  0 -1;
         0  1 -1  0;
         0 -1  0  1;
         1  0 -1  0;
         0  1  0 -1];
kronp=kron(E,I);
%Calculo del vector R
R=e*kronp

%% Estructura minimamente rigida
%Definir las p
e1=p2-p1;
e2=p4-p1;
e3=p3-p2;
e4=p3-p4;
e5=p3-p1;

e2=blkdiag([e1(1) e1(2)],[e2(1) e2(2)],[e3(1) e3(2)],[e4(1) e4(2)],[e5(1) e5(2)]);
%Matrices de incidencia locales cosa rigida
E2 = [1 -1  0  0;
              1  0  0 -1;
              0  1  0 -1;
              0  0 -1  1;
              1  0 -1  0];
kronp2=kron(E2,I);
%Calculo del vector R
R2=e2*kronp2

%% Estructura no rigida
%Definir las p
e1=p2-p1;
e2=p4-p1;
e3=p3-p2;
e4=p3-p4;

e3=blkdiag([e1(1) e1(2)],[e2(1) e2(2)],[e3(1) e3(2)],[e4(1) e4(2)]);
%Matrices de incidencia locales cosa rigida
E3 = [1 -1  0  0;
              1  0  0 -1;
              0  1 -1  0;
              0  0 -1  1];
kronp3=kron(E3,I);
%Calculo del vector R
R3=e3*kronp3