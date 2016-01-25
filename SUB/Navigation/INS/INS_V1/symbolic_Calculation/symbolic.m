
clear;
clc;

% Calib EKF
% syms Sx Sy Sz Mx My Mz bx by bz ix iy iz;
% E = [Sx     0       0;
%         Mz      Sy      0;
%         Mx      My      Sz];
% B = [bx;by;bz];
% I = [ix;iy;iz];
% h = (Sx*ix+bx)^2 + (Mz*ix+Sy*iy+by)^2 + (Mx*ix+My*iy+Sz*iz+bz)^2
% h2 = norm(E*(I+B))^2
% 
% dh_dSx = diff(h,Sx)
% dh_dSy = diff(h,Sy)
% dh_dSz = diff(h,Sz)
% dh_dMx = diff(h,Mx)
% dh_dMy = diff(h,My)
% dh_dMz = diff(h,Mz)
% dh_dbx = diff(h,bx)
% dh_dby = diff(h,by)
% dh_dbz = diff(h,bz)

%% Calib with multipos function Levenberg-Marquardt ACCEL
clear;
clc;

syms SFx SFy SFz Mx My Mz Bx By Bz REF;
syms RAWx1 RAWy1 RAWz1;
syms RAWx2 RAWy2 RAWz2;
syms RAWx3 RAWy3 RAWz3;
syms RAWx4 RAWy4 RAWz
syms RAWx5 RAWy5 RAWz5;
syms RAWx6 RAWy6 RAWz6;
syms RAWx7 RAWy7 RAWz7;
syms RAWx8 RAWy8 RAWz8;
syms RAWx9 RAWy9 RAWz9;
syms RAWx10 RAWy10 RAWz10;
syms RAWx11 RAWy11 RAWz11;
syms RAWx12 RAWy12 RAWz12;
syms RAWx13 RAWy13 RAWz13;
syms RAWx14 RAWy14 RAWz14;
syms RAWx15 RAWy15 RAWz15;
syms RAWx16 RAWy16 RAWz16;
syms RAWx17 RAWy17 RAWz17;
syms RAWx18 RAWy18 RAWz18;
syms RAWx19 RAWy19 RAWz19;
syms RAWx20 RAWy20 RAWz20;
syms RAWx21 RAWy21 RAWz21;
syms RAWx22 RAWy22 RAWz22;
syms RAWx23 RAWy23 RAWz23;
syms RAWx24 RAWy24 RAWz24;
syms RAWx25 RAWy25 RAWz25;
syms RAWx26 RAWy26 RAWz26;
% REF is ge

THETA = [SFx;SFy;SFz;Mx;My;Mz;Bx;By;Bz];

% h = norm^2 - ref^2
h(1) = ((SFx*RAWx1+Bx)^2 + (Mz*RAWx1+SFy*RAWy1+By)^2 + (Mx*RAWx1+My*RAWy1+SFz*RAWz1+Bz)^2)-REF^2;
h(2) = ((SFx*RAWx2+Bx)^2 + (Mz*RAWx2+SFy*RAWy2+By)^2 + (Mx*RAWx2+My*RAWy2+SFz*RAWz2+Bz)^2)-REF^2;
h(3) = ((SFx*RAWx3+Bx)^2 + (Mz*RAWx3+SFy*RAWy3+By)^2 + (Mx*RAWx3+My*RAWy3+SFz*RAWz3+Bz)^2)-REF^2;
h(4) = ((SFx*RAWx4+Bx)^2 + (Mz*RAWx4+SFy*RAWy4+By)^2 + (Mx*RAWx4+My*RAWy4+SFz*RAWz4+Bz)^2)-REF^2;
h(5) = ((SFx*RAWx5+Bx)^2 + (Mz*RAWx5+SFy*RAWy5+By)^2 + (Mx*RAWx5+My*RAWy5+SFz*RAWz5+Bz)^2)-REF^2;
h(6) = ((SFx*RAWx6+Bx)^2 + (Mz*RAWx6+SFy*RAWy6+By)^2 + (Mx*RAWx6+My*RAWy6+SFz*RAWz6+Bz)^2)-REF^2;
h(7) = ((SFx*RAWx7+Bx)^2 + (Mz*RAWx7+SFy*RAWy7+By)^2 + (Mx*RAWx7+My*RAWy7+SFz*RAWz7+Bz)^2)-REF^2;
h(8) = ((SFx*RAWx8+Bx)^2 + (Mz*RAWx8+SFy*RAWy8+By)^2 + (Mx*RAWx8+My*RAWy8+SFz*RAWz8+Bz)^2)-REF^2;
h(9) = ((SFx*RAWx9+Bx)^2 + (Mz*RAWx9+SFy*RAWy9+By)^2 + (Mx*RAWx9+My*RAWy9+SFz*RAWz9+Bz)^2)-REF^2;
h(10) = ((SFx*RAWx10+Bx)^2 + (Mz*RAWx10+SFy*RAWy10+By)^2 + (Mx*RAWx10+My*RAWy10+SFz*RAWz10+Bz)^2)-REF^2;
h(11) = ((SFx*RAWx11+Bx)^2 + (Mz*RAWx11+SFy*RAWy11+By)^2 + (Mx*RAWx11+My*RAWy11+SFz*RAWz11+Bz)^2)-REF^2;
h(12) = ((SFx*RAWx12+Bx)^2 + (Mz*RAWx12+SFy*RAWy12+By)^2 + (Mx*RAWx12+My*RAWy12+SFz*RAWz12+Bz)^2)-REF^2;
h(13) = ((SFx*RAWx13+Bx)^2 + (Mz*RAWx13+SFy*RAWy13+By)^2 + (Mx*RAWx13+My*RAWy13+SFz*RAWz13+Bz)^2)-REF^2;
h(14) = ((SFx*RAWx14+Bx)^2 + (Mz*RAWx14+SFy*RAWy14+By)^2 + (Mx*RAWx14+My*RAWy14+SFz*RAWz14+Bz)^2)-REF^2;
h(15) = ((SFx*RAWx15+Bx)^2 + (Mz*RAWx15+SFy*RAWy15+By)^2 + (Mx*RAWx15+My*RAWy15+SFz*RAWz15+Bz)^2)-REF^2;
h(16) = ((SFx*RAWx16+Bx)^2 + (Mz*RAWx16+SFy*RAWy16+By)^2 + (Mx*RAWx16+My*RAWy16+SFz*RAWz16+Bz)^2)-REF^2;
h(17) = ((SFx*RAWx17+Bx)^2 + (Mz*RAWx17+SFy*RAWy17+By)^2 + (Mx*RAWx17+My*RAWy17+SFz*RAWz17+Bz)^2)-REF^2;
h(18) = ((SFx*RAWx18+Bx)^2 + (Mz*RAWx18+SFy*RAWy18+By)^2 + (Mx*RAWx18+My*RAWy18+SFz*RAWz18+Bz)^2)-REF^2;
h(19) = ((SFx*RAWx19+Bx)^2 + (Mz*RAWx19+SFy*RAWy19+By)^2 + (Mx*RAWx19+My*RAWy19+SFz*RAWz19+Bz)^2)-REF^2;
h(20) = ((SFx*RAWx20+Bx)^2 + (Mz*RAWx20+SFy*RAWy20+By)^2 + (Mx*RAWx20+My*RAWy20+SFz*RAWz20+Bz)^2)-REF^2;
h(21) = ((SFx*RAWx21+Bx)^2 + (Mz*RAWx21+SFy*RAWy21+By)^2 + (Mx*RAWx21+My*RAWy21+SFz*RAWz21+Bz)^2)-REF^2;
h(22) = ((SFx*RAWx22+Bx)^2 + (Mz*RAWx22+SFy*RAWy22+By)^2 + (Mx*RAWx22+My*RAWy22+SFz*RAWz22+Bz)^2)-REF^2;
h(23) = ((SFx*RAWx23+Bx)^2 + (Mz*RAWx23+SFy*RAWy23+By)^2 + (Mx*RAWx23+My*RAWy23+SFz*RAWz23+Bz)^2)-REF^2;
h(24) = ((SFx*RAWx24+Bx)^2 + (Mz*RAWx24+SFy*RAWy24+By)^2 + (Mx*RAWx24+My*RAWy24+SFz*RAWz24+Bz)^2)-REF^2;
h(25) = ((SFx*RAWx25+Bx)^2 + (Mz*RAWx25+SFy*RAWy25+By)^2 + (Mx*RAWx25+My*RAWy25+SFz*RAWz25+Bz)^2)-REF^2;
h(26) = ((SFx*RAWx26+Bx)^2 + (Mz*RAWx26+SFy*RAWy26+By)^2 + (Mx*RAWx26+My*RAWy26+SFz*RAWz26+Bz)^2)-REF^2;
htot = 0.5*sum(h.^2)

J = jacobian(h,THETA) % 26*9 matrix
J1=J(1)
J2=J(2)
J3=J(3)
J4=J(4)
J5=J(5)
J6=J(6)
J7=J(7)
J8=J(8)
J9=J(9)

%% EKF GYRO
clear;
clc;

syms ux uy uz wx wy wz;
syms Sx Sy Sz Mx My Mz Tx Ty Tz bx by bz;
E = [Sx     Tz      Ty;
     Mz     Sy      Tx;
     Mx     My      Sz];
B = [bx;by;bz];
W = [wx;wy;wz];
U = [ux;uy;uz];
THETA = [Sx;Sy;Sz;Mx;My;Mz;Tx;Ty;Tz;bx;by;bz];

h(1,1) = uy*(Mx*wx+My*wy+Sz*wz+bz) - uz*(Mz*wx+Sy*wy+Tx*wz+by);
h(2,1) = uz*(Sx*wx+Tz*wy+Ty*wz+bx) - ux*(Mx*wx+My*wy+Sz*wz+bz);
h(3,1) = ux*(Mz*wx+Sy*wy+Tx*wz+by) - uy*(Sx*wx+Tz*wy+Ty*wz+bx);
h
h2 = cross(U,E*W+B)

J = jacobian(h,THETA)

dh_dSx = diff(h,Sx)
dh_dSy = diff(h,Sy)
dh_dSz = diff(h,Sz)
dh_dMx = diff(h,Mx)
dh_dMy = diff(h,My)
dh_dMz = diff(h,Mz)
dh_dTx = diff(h,Tx)
dh_dTy = diff(h,Ty)
dh_dTz = diff(h,Tz)
dh_dbx = diff(h,bx)
dh_dby = diff(h,by)
dh_dbz = diff(h,bz)

%% Calib with multipos function Levenberg-Marquardt MAG
clear;
clc;

syms SFx SFy SFz Mx My Mz Tx Ty Tz Bx By Bz REFnorm REFcross REFdot;
syms RAWx1 RAWy1 RAWz1 REFx1 REFy1 REFz1;
syms RAWx2 RAWy2 RAWz2 REFx2 REFy2 REFz2;
syms RAWx3 RAWy3 RAWz3 REFx3 REFy3 REFz3;
syms RAWx4 RAWy4 RAWz4 REFx4 REFy4 REFz4;
syms RAWx5 RAWy5 RAWz5 REFx5 REFy5 REFz5;
syms RAWx6 RAWy6 RAWz6 REFx6 REFy6 REFz6;
syms RAWx7 RAWy7 RAWz7 REFx7 REFy7 REFz7;
syms RAWx8 RAWy8 RAWz8 REFx8 REFy8 REFz8;
syms RAWx9 RAWy9 RAWz9 REFx9 REFy9 REFz9;
syms RAWx10 RAWy10 RAWz10 REFx10 REFy10 REFz10;
syms RAWx11 RAWy11 RAWz11 REFx11 REFy11 REFz11;
syms RAWx12 RAWy12 RAWz12 REFx12 REFy12 REFz12;
syms RAWx13 RAWy13 RAWz13 REFx13 REFy13 REFz13;
syms RAWx14 RAWy14 RAWz14 REFx14 REFy14 REFz14;
syms RAWx15 RAWy15 RAWz15 REFx15 REFy15 REFz15;
syms RAWx16 RAWy16 RAWz16 REFx16 REFy16 REFz16;
syms RAWx17 RAWy17 RAWz17 REFx17 REFy17 REFz17;
syms RAWx18 RAWy18 RAWz18 REFx18 REFy18 REFz18;
syms RAWx19 RAWy19 RAWz19 REFx19 REFy19 REFz19;
syms RAWx20 RAWy20 RAWz20 REFx20 REFy20 REFz20;
syms RAWx21 RAWy21 RAWz21 REFx21 REFy21 REFz21;
syms RAWx22 RAWy22 RAWz22 REFx22 REFy22 REFz22;
syms RAWx23 RAWy23 RAWz23 REFx23 REFy23 REFz23;
syms RAWx24 RAWy24 RAWz24 REFx24 REFy24 REFz24;
syms RAWx25 RAWy25 RAWz25 REFx25 REFy25 REFz25;
syms RAWx26 RAWy26 RAWz26 REFx26 REFy26 REFz26;
% REF is meTOT/ge*Rot_mg*CALIBRATED(acc_x,accy,accz)

THETA = [SFx;SFy;SFz;Mx;My;Mz;Bx;By;Bz;Tx;Ty;Tz];

% h = norm^2 - ref^2
h(1) = (SFx*RAWx1+Tz*RAWy1+Ty*RAWz1+Bx)^2 + (Mz*RAWx1+SFy*RAWy1+Tx*RAWz1+By)^2 + (Mx*RAWx1+My*RAWy1+SFz*RAWz1+Bz)^2 - REFnorm^2;
h(2) = (SFx*RAWx2+Tz*RAWy2+Ty*RAWz2+Bx)^2 + (Mz*RAWx2+SFy*RAWy2+Tx*RAWz2+By)^2 + (Mx*RAWx2+My*RAWy2+SFz*RAWz2+Bz)^2 - REFnorm^2;
h(3) = (SFx*RAWx3+Tz*RAWy3+Ty*RAWz3+Bx)^2 + (Mz*RAWx3+SFy*RAWy3+Tx*RAWz3+By)^2 + (Mx*RAWx3+My*RAWy3+SFz*RAWz3+Bz)^2 - REFnorm^2;
h(4) = (SFx*RAWx4+Tz*RAWy4+Ty*RAWz4+Bx)^2 + (Mz*RAWx4+SFy*RAWy4+Tx*RAWz4+By)^2 + (Mx*RAWx4+My*RAWy4+SFz*RAWz4+Bz)^2 - REFnorm^2;
h(5) = (SFx*RAWx5+Tz*RAWy5+Ty*RAWz5+Bx)^2 + (Mz*RAWx5+SFy*RAWy5+Tx*RAWz5+By)^2 + (Mx*RAWx5+My*RAWy5+SFz*RAWz5+Bz)^2 - REFnorm^2;
h(6) = (SFx*RAWx6+Tz*RAWy6+Ty*RAWz6+Bx)^2 + (Mz*RAWx6+SFy*RAWy6+Tx*RAWz6+By)^2 + (Mx*RAWx6+My*RAWy6+SFz*RAWz6+Bz)^2 - REFnorm^2;
h(7) = (SFx*RAWx7+Tz*RAWy7+Ty*RAWz7+Bx)^2 + (Mz*RAWx7+SFy*RAWy7+Tx*RAWz7+By)^2 + (Mx*RAWx7+My*RAWy7+SFz*RAWz7+Bz)^2 - REFnorm^2;
h(8) = (SFx*RAWx8+Tz*RAWy8+Ty*RAWz8+Bx)^2 + (Mz*RAWx8+SFy*RAWy8+Tx*RAWz8+By)^2 + (Mx*RAWx8+My*RAWy8+SFz*RAWz8+Bz)^2 - REFnorm^2;
h(9) = (SFx*RAWx9+Tz*RAWy9+Ty*RAWz9+Bx)^2 + (Mz*RAWx9+SFy*RAWy9+Tx*RAWz9+By)^2 + (Mx*RAWx9+My*RAWy9+SFz*RAWz9+Bz)^2 - REFnorm^2;
h(10) = (SFx*RAWx10+Tz*RAWy10+Ty*RAWz10+Bx)^2 + (Mz*RAWx10+SFy*RAWy10+Tx*RAWz10+By)^2 + (Mx*RAWx10+My*RAWy10+SFz*RAWz10+Bz)^2 - REFnorm^2;
h(11) = (SFx*RAWx11+Tz*RAWy11+Ty*RAWz11+Bx)^2 + (Mz*RAWx11+SFy*RAWy11+Tx*RAWz11+By)^2 + (Mx*RAWx11+My*RAWy11+SFz*RAWz11+Bz)^2 - REFnorm^2;
h(12) = (SFx*RAWx12+Tz*RAWy12+Ty*RAWz12+Bx)^2 + (Mz*RAWx12+SFy*RAWy12+Tx*RAWz12+By)^2 + (Mx*RAWx12+My*RAWy12+SFz*RAWz12+Bz)^2 - REFnorm^2;
h(13) = (SFx*RAWx13+Tz*RAWy13+Ty*RAWz13+Bx)^2 + (Mz*RAWx13+SFy*RAWy13+Tx*RAWz13+By)^2 + (Mx*RAWx13+My*RAWy13+SFz*RAWz13+Bz)^2 - REFnorm^2;
h(14) = (SFx*RAWx14+Tz*RAWy14+Ty*RAWz14+Bx)^2 + (Mz*RAWx14+SFy*RAWy14+Tx*RAWz14+By)^2 + (Mx*RAWx14+My*RAWy14+SFz*RAWz14+Bz)^2 - REFnorm^2;
h(15) = (SFx*RAWx15+Tz*RAWy15+Ty*RAWz15+Bx)^2 + (Mz*RAWx15+SFy*RAWy15+Tx*RAWz15+By)^2 + (Mx*RAWx15+My*RAWy15+SFz*RAWz15+Bz)^2 - REFnorm^2;
h(16) = (SFx*RAWx16+Tz*RAWy16+Ty*RAWz16+Bx)^2 + (Mz*RAWx16+SFy*RAWy16+Tx*RAWz16+By)^2 + (Mx*RAWx16+My*RAWy16+SFz*RAWz16+Bz)^2 - REFnorm^2;
h(17) = (SFx*RAWx17+Tz*RAWy17+Ty*RAWz17+Bx)^2 + (Mz*RAWx17+SFy*RAWy17+Tx*RAWz17+By)^2 + (Mx*RAWx17+My*RAWy17+SFz*RAWz17+Bz)^2 - REFnorm^2;
h(18) = (SFx*RAWx18+Tz*RAWy18+Ty*RAWz18+Bx)^2 + (Mz*RAWx18+SFy*RAWy18+Tx*RAWz18+By)^2 + (Mx*RAWx18+My*RAWy18+SFz*RAWz18+Bz)^2 - REFnorm^2;
h(19) = (SFx*RAWx19+Tz*RAWy19+Ty*RAWz19+Bx)^2 + (Mz*RAWx19+SFy*RAWy19+Tx*RAWz19+By)^2 + (Mx*RAWx19+My*RAWy19+SFz*RAWz19+Bz)^2 - REFnorm^2;
h(20) = (SFx*RAWx20+Tz*RAWy20+Ty*RAWz20+Bx)^2 + (Mz*RAWx20+SFy*RAWy20+Tx*RAWz20+By)^2 + (Mx*RAWx20+My*RAWy20+SFz*RAWz20+Bz)^2 - REFnorm^2;
h(21) = (SFx*RAWx21+Tz*RAWy21+Ty*RAWz21+Bx)^2 + (Mz*RAWx21+SFy*RAWy21+Tx*RAWz21+By)^2 + (Mx*RAWx21+My*RAWy21+SFz*RAWz21+Bz)^2 - REFnorm^2;
h(22) = (SFx*RAWx22+Tz*RAWy22+Ty*RAWz22+Bx)^2 + (Mz*RAWx22+SFy*RAWy22+Tx*RAWz22+By)^2 + (Mx*RAWx22+My*RAWy22+SFz*RAWz22+Bz)^2 - REFnorm^2;
h(23) = (SFx*RAWx23+Tz*RAWy23+Ty*RAWz23+Bx)^2 + (Mz*RAWx23+SFy*RAWy23+Tx*RAWz23+By)^2 + (Mx*RAWx23+My*RAWy23+SFz*RAWz23+Bz)^2 - REFnorm^2;
h(24) = (SFx*RAWx24+Tz*RAWy24+Ty*RAWz24+Bx)^2 + (Mz*RAWx24+SFy*RAWy24+Tx*RAWz24+By)^2 + (Mx*RAWx24+My*RAWy24+SFz*RAWz24+Bz)^2 - REFnorm^2;
h(25) = (SFx*RAWx25+Tz*RAWy25+Ty*RAWz25+Bx)^2 + (Mz*RAWx25+SFy*RAWy25+Tx*RAWz25+By)^2 + (Mx*RAWx25+My*RAWy25+SFz*RAWz25+Bz)^2 - REFnorm^2;
h(26) = (SFx*RAWx26+Tz*RAWy26+Ty*RAWz26+Bx)^2 + (Mz*RAWx26+SFy*RAWy26+Tx*RAWz26+By)^2 + (Mx*RAWx26+My*RAWy26+SFz*RAWz26+Bz)^2 - REFnorm^2;
% h(27) = (REFy1*(Mx*RAWx1+My*RAWy1+SFz*RAWz1+Bz)-REFz1*(Mz*RAWx1+SFy*RAWy1+Tx*RAWz1+By))^2 + (REFz1*(SFx*RAWx1+Tz*RAWy1+Ty*RAWz1+Bx) - REFx1*(Mx*RAWx1+My*RAWy1+SFz*RAWz1+Bz))^2 + (REFx1*(Mz*RAWx1+SFy*RAWy1+Tx*RAWz1+By) - REFy1*(SFx*RAWx1+Tz*RAWy1+Ty*RAWz1+Bx))^2 - REFcross^2;
% h(28) = (REFy2*(Mx*RAWx2+My*RAWy2+SFz*RAWz2+Bz)-REFz2*(Mz*RAWx2+SFy*RAWy2+Tx*RAWz2+By))^2 + (REFz2*(SFx*RAWx2+Tz*RAWy2+Ty*RAWz2+Bx) - REFx2*(Mx*RAWx2+My*RAWy2+SFz*RAWz2+Bz))^2 + (REFx2*(Mz*RAWx2+SFy*RAWy2+Tx*RAWz2+By) - REFy2*(SFx*RAWx2+Tz*RAWy2+Ty*RAWz2+Bx))^2 - REFcross^2;
% h(29) = (REFy3*(Mx*RAWx3+My*RAWy3+SFz*RAWz3+Bz)-REFz3*(Mz*RAWx3+SFy*RAWy3+Tx*RAWz3+By))^2 + (REFz3*(SFx*RAWx3+Tz*RAWy3+Ty*RAWz3+Bx) - REFx3*(Mx*RAWx3+My*RAWy3+SFz*RAWz3+Bz))^2 + (REFx3*(Mz*RAWx3+SFy*RAWy3+Tx*RAWz3+By) - REFy3*(SFx*RAWx3+Tz*RAWy3+Ty*RAWz3+Bx))^2 - REFcross^2;
% h(30) = (REFy4*(Mx*RAWx4+My*RAWy4+SFz*RAWz4+Bz)-REFz4*(Mz*RAWx4+SFy*RAWy4+Tx*RAWz4+By))^2 + (REFz4*(SFx*RAWx4+Tz*RAWy4+Ty*RAWz4+Bx) - REFx4*(Mx*RAWx4+My*RAWy4+SFz*RAWz4+Bz))^2 + (REFx4*(Mz*RAWx4+SFy*RAWy4+Tx*RAWz4+By) - REFy4*(SFx*RAWx4+Tz*RAWy4+Ty*RAWz4+Bx))^2 - REFcross^2;
% h(31) = (REFy5*(Mx*RAWx5+My*RAWy5+SFz*RAWz5+Bz)-REFz5*(Mz*RAWx5+SFy*RAWy5+Tx*RAWz5+By))^2 + (REFz5*(SFx*RAWx5+Tz*RAWy5+Ty*RAWz5+Bx) - REFx5*(Mx*RAWx5+My*RAWy5+SFz*RAWz5+Bz))^2 + (REFx5*(Mz*RAWx5+SFy*RAWy5+Tx*RAWz5+By) - REFy5*(SFx*RAWx5+Tz*RAWy5+Ty*RAWz5+Bx))^2 - REFcross^2;
% h(32) = (REFy6*(Mx*RAWx6+My*RAWy6+SFz*RAWz6+Bz)-REFz6*(Mz*RAWx6+SFy*RAWy6+Tx*RAWz6+By))^2 + (REFz6*(SFx*RAWx6+Tz*RAWy6+Ty*RAWz6+Bx) - REFx6*(Mx*RAWx6+My*RAWy6+SFz*RAWz6+Bz))^2 + (REFx6*(Mz*RAWx6+SFy*RAWy6+Tx*RAWz6+By) - REFy6*(SFx*RAWx6+Tz*RAWy6+Ty*RAWz6+Bx))^2 - REFcross^2;
% h(33) = (REFy7*(Mx*RAWx7+My*RAWy7+SFz*RAWz7+Bz)-REFz7*(Mz*RAWx7+SFy*RAWy7+Tx*RAWz7+By))^2 + (REFz7*(SFx*RAWx7+Tz*RAWy7+Ty*RAWz7+Bx) - REFx7*(Mx*RAWx7+My*RAWy7+SFz*RAWz7+Bz))^2 + (REFx7*(Mz*RAWx7+SFy*RAWy7+Tx*RAWz7+By) - REFy7*(SFx*RAWx7+Tz*RAWy7+Ty*RAWz7+Bx))^2 - REFcross^2;
% h(34) = (REFy8*(Mx*RAWx8+My*RAWy8+SFz*RAWz8+Bz)-REFz8*(Mz*RAWx8+SFy*RAWy8+Tx*RAWz8+By))^2 + (REFz8*(SFx*RAWx8+Tz*RAWy8+Ty*RAWz8+Bx) - REFx8*(Mx*RAWx8+My*RAWy8+SFz*RAWz8+Bz))^2 + (REFx8*(Mz*RAWx8+SFy*RAWy8+Tx*RAWz8+By) - REFy8*(SFx*RAWx8+Tz*RAWy8+Ty*RAWz8+Bx))^2 - REFcross^2;
% h(35) = (REFy9*(Mx*RAWx9+My*RAWy9+SFz*RAWz9+Bz)-REFz9*(Mz*RAWx9+SFy*RAWy9+Tx*RAWz9+By))^2 + (REFz9*(SFx*RAWx9+Tz*RAWy9+Ty*RAWz9+Bx) - REFx9*(Mx*RAWx9+My*RAWy9+SFz*RAWz9+Bz))^2 + (REFx9*(Mz*RAWx9+SFy*RAWy9+Tx*RAWz9+By) - REFy9*(SFx*RAWx9+Tz*RAWy9+Ty*RAWz9+Bx))^2 - REFcross^2;
% h(36) = (REFy10*(Mx*RAWx10+My*RAWy10+SFz*RAWz10+Bz)-REFz10*(Mz*RAWx10+SFy*RAWy10+Tx*RAWz10+By))^2 + (REFz10*(SFx*RAWx10+Tz*RAWy10+Ty*RAWz10+Bx) - REFx10*(Mx*RAWx10+My*RAWy10+SFz*RAWz10+Bz))^2 + (REFx10*(Mz*RAWx10+SFy*RAWy10+Tx*RAWz10+By) - REFy10*(SFx*RAWx10+Tz*RAWy10+Ty*RAWz10+Bx))^2 - REFcross^2;
% h(37) = (REFy11*(Mx*RAWx11+My*RAWy11+SFz*RAWz11+Bz)-REFz11*(Mz*RAWx11+SFy*RAWy11+Tx*RAWz11+By))^2 + (REFz11*(SFx*RAWx11+Tz*RAWy11+Ty*RAWz11+Bx) - REFx11*(Mx*RAWx11+My*RAWy11+SFz*RAWz11+Bz))^2 + (REFx11*(Mz*RAWx11+SFy*RAWy11+Tx*RAWz11+By) - REFy11*(SFx*RAWx11+Tz*RAWy11+Ty*RAWz11+Bx))^2 - REFcross^2;
% h(38) = (REFy12*(Mx*RAWx12+My*RAWy12+SFz*RAWz12+Bz)-REFz12*(Mz*RAWx12+SFy*RAWy12+Tx*RAWz12+By))^2 + (REFz12*(SFx*RAWx12+Tz*RAWy12+Ty*RAWz12+Bx) - REFx12*(Mx*RAWx12+My*RAWy12+SFz*RAWz12+Bz))^2 + (REFx12*(Mz*RAWx12+SFy*RAWy12+Tx*RAWz12+By) - REFy12*(SFx*RAWx12+Tz*RAWy12+Ty*RAWz12+Bx))^2 - REFcross^2;
% h(39) = (REFy13*(Mx*RAWx13+My*RAWy13+SFz*RAWz13+Bz)-REFz13*(Mz*RAWx13+SFy*RAWy13+Tx*RAWz13+By))^2 + (REFz13*(SFx*RAWx13+Tz*RAWy13+Ty*RAWz13+Bx) - REFx13*(Mx*RAWx13+My*RAWy13+SFz*RAWz13+Bz))^2 + (REFx13*(Mz*RAWx13+SFy*RAWy13+Tx*RAWz13+By) - REFy13*(SFx*RAWx13+Tz*RAWy13+Ty*RAWz13+Bx))^2 - REFcross^2;
% h(40) = (REFy14*(Mx*RAWx14+My*RAWy14+SFz*RAWz14+Bz)-REFz14*(Mz*RAWx14+SFy*RAWy14+Tx*RAWz14+By))^2 + (REFz14*(SFx*RAWx14+Tz*RAWy14+Ty*RAWz14+Bx) - REFx14*(Mx*RAWx14+My*RAWy14+SFz*RAWz14+Bz))^2 + (REFx14*(Mz*RAWx14+SFy*RAWy14+Tx*RAWz14+By) - REFy14*(SFx*RAWx14+Tz*RAWy14+Ty*RAWz14+Bx))^2 - REFcross^2;
% h(41) = (REFy15*(Mx*RAWx15+My*RAWy15+SFz*RAWz15+Bz)-REFz15*(Mz*RAWx15+SFy*RAWy15+Tx*RAWz15+By))^2 + (REFz15*(SFx*RAWx15+Tz*RAWy15+Ty*RAWz15+Bx) - REFx15*(Mx*RAWx15+My*RAWy15+SFz*RAWz15+Bz))^2 + (REFx15*(Mz*RAWx15+SFy*RAWy15+Tx*RAWz15+By) - REFy15*(SFx*RAWx15+Tz*RAWy15+Ty*RAWz15+Bx))^2 - REFcross^2;
% h(42) = (REFy16*(Mx*RAWx16+My*RAWy16+SFz*RAWz16+Bz)-REFz16*(Mz*RAWx16+SFy*RAWy16+Tx*RAWz16+By))^2 + (REFz16*(SFx*RAWx16+Tz*RAWy16+Ty*RAWz16+Bx) - REFx16*(Mx*RAWx16+My*RAWy16+SFz*RAWz16+Bz))^2 + (REFx16*(Mz*RAWx16+SFy*RAWy16+Tx*RAWz16+By) - REFy16*(SFx*RAWx16+Tz*RAWy16+Ty*RAWz16+Bx))^2 - REFcross^2;
% h(43) = (REFy17*(Mx*RAWx17+My*RAWy17+SFz*RAWz17+Bz)-REFz17*(Mz*RAWx17+SFy*RAWy17+Tx*RAWz17+By))^2 + (REFz17*(SFx*RAWx17+Tz*RAWy17+Ty*RAWz17+Bx) - REFx17*(Mx*RAWx17+My*RAWy17+SFz*RAWz17+Bz))^2 + (REFx17*(Mz*RAWx17+SFy*RAWy17+Tx*RAWz17+By) - REFy17*(SFx*RAWx17+Tz*RAWy17+Ty*RAWz17+Bx))^2 - REFcross^2;
% h(44) = (REFy18*(Mx*RAWx18+My*RAWy18+SFz*RAWz18+Bz)-REFz18*(Mz*RAWx18+SFy*RAWy18+Tx*RAWz18+By))^2 + (REFz18*(SFx*RAWx18+Tz*RAWy18+Ty*RAWz18+Bx) - REFx18*(Mx*RAWx18+My*RAWy18+SFz*RAWz18+Bz))^2 + (REFx18*(Mz*RAWx18+SFy*RAWy18+Tx*RAWz18+By) - REFy18*(SFx*RAWx18+Tz*RAWy18+Ty*RAWz18+Bx))^2 - REFcross^2;
% h(45) = (REFy19*(Mx*RAWx19+My*RAWy19+SFz*RAWz19+Bz)-REFz19*(Mz*RAWx19+SFy*RAWy19+Tx*RAWz19+By))^2 + (REFz19*(SFx*RAWx19+Tz*RAWy19+Ty*RAWz19+Bx) - REFx19*(Mx*RAWx19+My*RAWy19+SFz*RAWz19+Bz))^2 + (REFx19*(Mz*RAWx19+SFy*RAWy19+Tx*RAWz19+By) - REFy19*(SFx*RAWx19+Tz*RAWy19+Ty*RAWz19+Bx))^2 - REFcross^2;
% h(46) = (REFy20*(Mx*RAWx20+My*RAWy20+SFz*RAWz20+Bz)-REFz20*(Mz*RAWx20+SFy*RAWy20+Tx*RAWz20+By))^2 + (REFz20*(SFx*RAWx20+Tz*RAWy20+Ty*RAWz20+Bx) - REFx20*(Mx*RAWx20+My*RAWy20+SFz*RAWz20+Bz))^2 + (REFx20*(Mz*RAWx20+SFy*RAWy20+Tx*RAWz20+By) - REFy20*(SFx*RAWx20+Tz*RAWy20+Ty*RAWz20+Bx))^2 - REFcross^2;
% h(47) = (REFy21*(Mx*RAWx21+My*RAWy21+SFz*RAWz21+Bz)-REFz21*(Mz*RAWx21+SFy*RAWy21+Tx*RAWz21+By))^2 + (REFz21*(SFx*RAWx21+Tz*RAWy21+Ty*RAWz21+Bx) - REFx21*(Mx*RAWx21+My*RAWy21+SFz*RAWz21+Bz))^2 + (REFx21*(Mz*RAWx21+SFy*RAWy21+Tx*RAWz21+By) - REFy21*(SFx*RAWx21+Tz*RAWy21+Ty*RAWz21+Bx))^2 - REFcross^2;
% h(48) = (REFy22*(Mx*RAWx22+My*RAWy22+SFz*RAWz22+Bz)-REFz22*(Mz*RAWx22+SFy*RAWy22+Tx*RAWz22+By))^2 + (REFz22*(SFx*RAWx22+Tz*RAWy22+Ty*RAWz22+Bx) - REFx22*(Mx*RAWx22+My*RAWy22+SFz*RAWz22+Bz))^2 + (REFx22*(Mz*RAWx22+SFy*RAWy22+Tx*RAWz22+By) - REFy22*(SFx*RAWx22+Tz*RAWy22+Ty*RAWz22+Bx))^2 - REFcross^2;
% h(49) = (REFy23*(Mx*RAWx23+My*RAWy23+SFz*RAWz23+Bz)-REFz23*(Mz*RAWx23+SFy*RAWy23+Tx*RAWz23+By))^2 + (REFz23*(SFx*RAWx23+Tz*RAWy23+Ty*RAWz23+Bx) - REFx23*(Mx*RAWx23+My*RAWy23+SFz*RAWz23+Bz))^2 + (REFx23*(Mz*RAWx23+SFy*RAWy23+Tx*RAWz23+By) - REFy23*(SFx*RAWx23+Tz*RAWy23+Ty*RAWz23+Bx))^2 - REFcross^2;
% h(50) = (REFy24*(Mx*RAWx24+My*RAWy24+SFz*RAWz24+Bz)-REFz24*(Mz*RAWx24+SFy*RAWy24+Tx*RAWz24+By))^2 + (REFz24*(SFx*RAWx24+Tz*RAWy24+Ty*RAWz24+Bx) - REFx24*(Mx*RAWx24+My*RAWy24+SFz*RAWz24+Bz))^2 + (REFx24*(Mz*RAWx24+SFy*RAWy24+Tx*RAWz24+By) - REFy24*(SFx*RAWx24+Tz*RAWy24+Ty*RAWz24+Bx))^2 - REFcross^2;
% h(51) = (REFy25*(Mx*RAWx25+My*RAWy25+SFz*RAWz25+Bz)-REFz25*(Mz*RAWx25+SFy*RAWy25+Tx*RAWz25+By))^2 + (REFz25*(SFx*RAWx25+Tz*RAWy25+Ty*RAWz25+Bx) - REFx25*(Mx*RAWx25+My*RAWy25+SFz*RAWz25+Bz))^2 + (REFx25*(Mz*RAWx25+SFy*RAWy25+Tx*RAWz25+By) - REFy25*(SFx*RAWx25+Tz*RAWy25+Ty*RAWz25+Bx))^2 - REFcross^2;
% h(52) = (REFy26*(Mx*RAWx26+My*RAWy26+SFz*RAWz26+Bz)-REFz26*(Mz*RAWx26+SFy*RAWy26+Tx*RAWz26+By))^2 + (REFz26*(SFx*RAWx26+Tz*RAWy26+Ty*RAWz26+Bx) - REFx26*(Mx*RAWx26+My*RAWy26+SFz*RAWz26+Bz))^2 + (REFx26*(Mz*RAWx26+SFy*RAWy26+Tx*RAWz26+By) - REFy26*(SFx*RAWx26+Tz*RAWy26+Ty*RAWz26+Bx))^2 - REFcross^2;
h(27) = REFx1*(SFx*RAWx1+Tz*RAWy1+Ty*RAWz1+Bx) + REFy1*(Mz*RAWx1+SFy*RAWy1+Tx*RAWz1+By) + REFz1*(Mx*RAWx1+My*RAWy1+SFz*RAWz1+Bz) - REFdot;
h(28) = REFx2*(SFx*RAWx2+Tz*RAWy2+Ty*RAWz2+Bx) + REFy2*(Mz*RAWx2+SFy*RAWy2+Tx*RAWz2+By) + REFz2*(Mx*RAWx2+My*RAWy2+SFz*RAWz2+Bz) - REFdot;
h(29) = REFx3*(SFx*RAWx3+Tz*RAWy3+Ty*RAWz3+Bx) + REFy3*(Mz*RAWx3+SFy*RAWy3+Tx*RAWz3+By) + REFz3*(Mx*RAWx3+My*RAWy3+SFz*RAWz3+Bz) - REFdot;
h(30) = REFx4*(SFx*RAWx4+Tz*RAWy4+Ty*RAWz4+Bx) + REFy4*(Mz*RAWx4+SFy*RAWy4+Tx*RAWz4+By) + REFz4*(Mx*RAWx4+My*RAWy4+SFz*RAWz4+Bz) - REFdot;
h(31) = REFx5*(SFx*RAWx5+Tz*RAWy5+Ty*RAWz5+Bx) + REFy5*(Mz*RAWx5+SFy*RAWy5+Tx*RAWz5+By) + REFz5*(Mx*RAWx5+My*RAWy5+SFz*RAWz5+Bz) - REFdot;
h(32) = REFx6*(SFx*RAWx6+Tz*RAWy6+Ty*RAWz6+Bx) + REFy6*(Mz*RAWx6+SFy*RAWy6+Tx*RAWz6+By) + REFz6*(Mx*RAWx6+My*RAWy6+SFz*RAWz6+Bz) - REFdot;
h(33) = REFx7*(SFx*RAWx7+Tz*RAWy7+Ty*RAWz7+Bx) + REFy7*(Mz*RAWx7+SFy*RAWy7+Tx*RAWz7+By) + REFz7*(Mx*RAWx7+My*RAWy7+SFz*RAWz7+Bz) - REFdot;
h(34) = REFx8*(SFx*RAWx8+Tz*RAWy8+Ty*RAWz8+Bx) + REFy8*(Mz*RAWx8+SFy*RAWy8+Tx*RAWz8+By) + REFz8*(Mx*RAWx8+My*RAWy8+SFz*RAWz8+Bz) - REFdot;
h(35) = REFx9*(SFx*RAWx9+Tz*RAWy9+Ty*RAWz9+Bx) + REFy9*(Mz*RAWx9+SFy*RAWy9+Tx*RAWz9+By) + REFz9*(Mx*RAWx9+My*RAWy9+SFz*RAWz9+Bz) - REFdot;
h(36) = REFx10*(SFx*RAWx10+Tz*RAWy10+Ty*RAWz10+Bx) + REFy10*(Mz*RAWx10+SFy*RAWy10+Tx*RAWz10+By) + REFz10*(Mx*RAWx10+My*RAWy10+SFz*RAWz10+Bz) - REFdot;
h(37) = REFx11*(SFx*RAWx11+Tz*RAWy11+Ty*RAWz11+Bx) + REFy11*(Mz*RAWx11+SFy*RAWy11+Tx*RAWz11+By) + REFz11*(Mx*RAWx11+My*RAWy11+SFz*RAWz11+Bz) - REFdot;
h(38) = REFx12*(SFx*RAWx12+Tz*RAWy12+Ty*RAWz12+Bx) + REFy12*(Mz*RAWx12+SFy*RAWy12+Tx*RAWz12+By) + REFz12*(Mx*RAWx12+My*RAWy12+SFz*RAWz12+Bz) - REFdot;
h(39) = REFx13*(SFx*RAWx13+Tz*RAWy13+Ty*RAWz13+Bx) + REFy13*(Mz*RAWx13+SFy*RAWy13+Tx*RAWz13+By) + REFz13*(Mx*RAWx13+My*RAWy13+SFz*RAWz13+Bz) - REFdot;
h(40) = REFx14*(SFx*RAWx14+Tz*RAWy14+Ty*RAWz14+Bx) + REFy14*(Mz*RAWx14+SFy*RAWy14+Tx*RAWz14+By) + REFz14*(Mx*RAWx14+My*RAWy14+SFz*RAWz14+Bz) - REFdot;
h(41) = REFx15*(SFx*RAWx15+Tz*RAWy15+Ty*RAWz15+Bx) + REFy15*(Mz*RAWx15+SFy*RAWy15+Tx*RAWz15+By) + REFz15*(Mx*RAWx15+My*RAWy15+SFz*RAWz15+Bz) - REFdot;
h(42) = REFx16*(SFx*RAWx16+Tz*RAWy16+Ty*RAWz16+Bx) + REFy16*(Mz*RAWx16+SFy*RAWy16+Tx*RAWz16+By) + REFz16*(Mx*RAWx16+My*RAWy16+SFz*RAWz16+Bz) - REFdot;
h(43) = REFx17*(SFx*RAWx17+Tz*RAWy17+Ty*RAWz17+Bx) + REFy17*(Mz*RAWx17+SFy*RAWy17+Tx*RAWz17+By) + REFz17*(Mx*RAWx17+My*RAWy17+SFz*RAWz17+Bz) - REFdot;
h(44) = REFx18*(SFx*RAWx18+Tz*RAWy18+Ty*RAWz18+Bx) + REFy18*(Mz*RAWx18+SFy*RAWy18+Tx*RAWz18+By) + REFz18*(Mx*RAWx18+My*RAWy18+SFz*RAWz18+Bz) - REFdot;
h(45) = REFx19*(SFx*RAWx19+Tz*RAWy19+Ty*RAWz19+Bx) + REFy19*(Mz*RAWx19+SFy*RAWy19+Tx*RAWz19+By) + REFz19*(Mx*RAWx19+My*RAWy19+SFz*RAWz19+Bz) - REFdot;
h(46) = REFx20*(SFx*RAWx20+Tz*RAWy20+Ty*RAWz20+Bx) + REFy20*(Mz*RAWx20+SFy*RAWy20+Tx*RAWz20+By) + REFz20*(Mx*RAWx20+My*RAWy20+SFz*RAWz20+Bz) - REFdot;
h(47) = REFx21*(SFx*RAWx21+Tz*RAWy21+Ty*RAWz21+Bx) + REFy21*(Mz*RAWx21+SFy*RAWy21+Tx*RAWz21+By) + REFz21*(Mx*RAWx21+My*RAWy21+SFz*RAWz21+Bz) - REFdot;
h(48) = REFx22*(SFx*RAWx22+Tz*RAWy22+Ty*RAWz22+Bx) + REFy22*(Mz*RAWx22+SFy*RAWy22+Tx*RAWz22+By) + REFz22*(Mx*RAWx22+My*RAWy22+SFz*RAWz22+Bz) - REFdot;
h(49) = REFx23*(SFx*RAWx23+Tz*RAWy23+Ty*RAWz23+Bx) + REFy23*(Mz*RAWx23+SFy*RAWy23+Tx*RAWz23+By) + REFz23*(Mx*RAWx23+My*RAWy23+SFz*RAWz23+Bz) - REFdot;
h(50) = REFx24*(SFx*RAWx24+Tz*RAWy24+Ty*RAWz24+Bx) + REFy24*(Mz*RAWx24+SFy*RAWy24+Tx*RAWz24+By) + REFz24*(Mx*RAWx24+My*RAWy24+SFz*RAWz24+Bz) - REFdot;
h(51) = REFx25*(SFx*RAWx25+Tz*RAWy25+Ty*RAWz25+Bx) + REFy25*(Mz*RAWx25+SFy*RAWy25+Tx*RAWz25+By) + REFz25*(Mx*RAWx25+My*RAWy25+SFz*RAWz25+Bz) - REFdot;
h(52) = REFx26*(SFx*RAWx26+Tz*RAWy26+Ty*RAWz26+Bx) + REFy26*(Mz*RAWx26+SFy*RAWy26+Tx*RAWz26+By) + REFz26*(Mx*RAWx26+My*RAWy26+SFz*RAWz26+Bz) - REFdot;
htot = 0.5*sum(h.^2)

J = jacobian(h,THETA) % 26*9 matrix
J1=J(1)
J2=J(2)
J3=J(3)
J4=J(4)
J5=J(5)
J6=J(6)
J7=J(7)
J8=J(8)
J9=J(9)
