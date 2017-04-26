%% Design of LQR controller for the Hexacopter

%Tuning constants according to Bryson's Rule
%Max values of the states:
maxn = 0.05;
maxe = 0.05;
maxd = 0.05;%0.25;
mndot = 0.1;%0.08;
mddot = 0.18;
mpsi = 10*pi/180;

% Sample 
Ts = 0.05;

%Maximum inputs (in acceleration)
tmax = 1.7;%15;%160;
mangle = 0.20;%0.15

%States are: n,e,d,ndot,edot,ddot,psi
A = zeros(7);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

%Inputs are: pitch, roll, thrust, yaw-dot
B = zeros(7,4);
B(4,1) = 1;%u_hover;
B(5,2) = 1;%-u_hover;
B(6,3) = 1;
B(7,4) = 1;

%Assuming full-state feedback (observing all the control states) here: 
C = eye(7);

% Place the maximum values for the states here
Q = eye(7);
Q(1,1) = 1/maxn^2;
Q(2,2) = 1/maxe^2;
Q(3,3) = 1/maxd*2;
Q(4,4) = 1/mndot^2;
Q(5,5) = 1/mndot^2;
Q(6,6) = 1/mddot^2;
Q(7,7) = 1/mpsi^2;

% Place the max values for the inputs here [pitch;roll;thrust;yaw-dot]
R = eye(4);
R(1,1) = 1/mangle^2;
R(2,2) = 1/mangle^2;
R(3,3) = 1/tmax^2;
R(4,4) = 1/mangle^2;

%Compute the gains, etc. 
[K,P,E] = lqr(A,B,Q,R);
[Kd,Pd,Ed] = lqrd(A,B,Q,R,Ts);
%[Kd,Pd,Ed] = dlqr(A,B,Q,R)

% Now add an integrator: Append the state with the integral state (int_n, int_e, int_d, int_psi)

%Augmented dynamics:
D = [eye(3,3) zeros(3,4)
     zeros(1,6) 1];

Abar = [A zeros(7,4)
        D zeros(4,4)];

Bbar = [B;zeros(4,4)];

%Place max values for the integral states here:
Q1 = [3 0 0 0;0 3 0 0;0 0 1 0;0 0 0 1];

Qz = [zeros(7,7) zeros(7,4)
      zeros(4,7) Q1];

%Place max values for inputs here (in regards to integral states)
Rv = [10 0 0 0;0 10 0 0; 0 0 10 0;0 0 0 10];

%Generate the gains:
[Kint,Pint,Eint] = lqr(Abar,Bbar,Qz,Rv);

Kint2 = zeros(size(K));
Kint2(:,1:4) = Kint(:,8:11);

%Put the gains into one matrix:
display('States are: n,e,d,ndot,edot,ddot,psi');
KTotal = [Kd;Kint2]

%%

%Write this gain matrix to a file:
dlmwrite('HeavyGainMatrices.txt', KTotal, 'delimiter', '\n', 'newline', 'pc','precision', 11);


