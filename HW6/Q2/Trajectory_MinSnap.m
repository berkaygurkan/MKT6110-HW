%   MKT6110 Autonomous Robots HW6 - Min Snap Trajectory      %
%   Author : Berkay GURKAN                                   %
clear all
%% Timestamps [t x]

Plan = [0 0 ;
         0.1452 0.2903;
         0.3210 0.6420;
         0.4919 0.9837;
         0.6211 1.2423;
         0.779  1.5558;
        ]
    
 
%% Point to Point Minimum Snap Trajectory 

% Boundary Conditions
V_Start = 0;        % Initial velocity
V_Final = 0;        % Final velocity
A_Start = 0;        % Initial acceleration       
A_Final = 0;        % Final acceleration  
J_Start = 0;        % Initial jerk  
J_Final = 0;        % Initial jerk  
V_Road=2;           % Desired velocity on the road


% Symbols for solver
syms c7 c6 c5 c4 c3 c2 c1 c0 x(t) v(t) a(t) j(t) x1(t) x2(t) x3(t) x4(t) x5(t)


% P1 to P2 
% Minimum Snap Equations
x(t) = c7*t^7 + c6*t^6 + c5*t^5+c4*t^4+c3*t^3+c2*t^2+c1*t+c0;
v(t) = 7*c7*t^6 + 6*c6*t^5 + 5*c5*t^4 + 4*c4*t^3 + 3*c3*t^2 + 2*c2*t + c1
a(t) = 42*c7*t^5 + 30*c6*t^4 + 20*c5*t^3 + 12*c4*t^2 +6*c3*t + 2*c2
j(t) = 210*c7*t^4 + 120*c6*t^3 + 60*c5*t^2 + 24*c4*t + 6*c3 ;

% Boundary Conditions 
eqn1 = x(0) == 0;
eqn2 = x(Plan(2,1)) == Plan(2,2);
eqn3 = v(0) == V_Start;
eqn4 = v(Plan(2,1)) == V_Road;
eqn5 = a(0) == 0;
eqn6 = a(Plan(2,1)) == 0;
eqn7 = j(0) == 0;
eqn8 = j(Plan(2,1)) == 0;
eqns = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6 eqn7 eqn8];

% Solver
S = solve(eqns);

x1(t)=subs(x(t),c0,double(S.c0));
x1(t)=subs(x1(t),c1,double(S.c1));
x1(t)=subs(x1(t),c2,double(S.c2));
x1(t)=subs(x1(t),c3,double(S.c3));
x1(t)=subs(x1(t),c4,double(S.c4));
x1(t)=subs(x1(t),c5,double(S.c5));
x1(t)=subs(x1(t),c6,double(S.c6));
x1(t)=subs(x1(t),c7,double(S.c7));


v1(t)=subs(v(t),c0,double(S.c0));
v1(t)=subs(v1(t),c1,double(S.c1));
v1(t)=subs(v1(t),c2,double(S.c2));
v1(t)=subs(v1(t),c3,double(S.c3));
v1(t)=subs(v1(t),c4,double(S.c4));
v1(t)=subs(v1(t),c5,double(S.c5));
v1(t)=subs(v1(t),c6,double(S.c6));
v1(t)=subs(v1(t),c7,double(S.c7));


a1(t)=subs(a(t),c0,double(S.c0));
a1(t)=subs(a1(t),c1,double(S.c1));
a1(t)=subs(a1(t),c2,double(S.c2));
a1(t)=subs(a1(t),c3,double(S.c3));
a1(t)=subs(a1(t),c4,double(S.c4));
a1(t)=subs(a1(t),c5,double(S.c5));
a1(t)=subs(a1(t),c6,double(S.c6));
a1(t)=subs(a1(t),c7,double(S.c7));

j1(t)=subs(j(t),c0,double(S.c0));
j1(t)=subs(j1(t),c1,double(S.c1));
j1(t)=subs(j1(t),c2,double(S.c2));
j1(t)=subs(j1(t),c3,double(S.c3));
j1(t)=subs(j1(t),c4,double(S.c4));
j1(t)=subs(j1(t),c5,double(S.c5));
j1(t)=subs(j1(t),c6,double(S.c6));
j1(t)=subs(j1(t),c7,double(S.c7));



% P2 to P3
% Minimum Snap Equations
x(t) = c7*(t-Plan(2,1))^7 + c6*(t-Plan(2,1))^6 + c5*(t-Plan(2,1))^5+c4*(t-Plan(2,1))^4+c3*(t-Plan(2,1))^3+c2*(t-Plan(2,1))^2+c1*(t-Plan(2,1))+c0;
v(t) = 7*c7*(t-Plan(2,1))^6 + 6*c6*(t-Plan(2,1))^5 + 5*c5*(t-Plan(2,1))^4 + 4*c4*(t-Plan(2,1))^3 + 3*c3*(t-Plan(2,1))^2 + 2*c2*(t-Plan(2,1)) + c1
a(t) = 42*c7*(t-Plan(2,1))^5 + 30*c6*(t-Plan(2,1))^4 + 20*c5*(t-Plan(2,1))^3 + 12*c4*(t-Plan(2,1))^2 +6*c3*(t-Plan(2,1)) + 2*c2
j(t) = 210*c7*(t-Plan(2,1))^4 + 120*c6*(t-Plan(2,1))^3 + 60*c5*(t-Plan(2,1))^2 + 24*c4*(t-Plan(2,1)) + 6*c3 ;

% Boundary Equations
eqn1 = x(Plan(2,1)) == Plan(2,2);
eqn2 = x(Plan(3,1)) == Plan(3,2);
eqn3 = a(Plan(2,1)) == 0;
eqn4 = a(Plan(3,1)) == 0;
eqn5 = v(Plan(2,1)) == V_Road;
eqn6 = v(Plan(3,1)) == V_Road;
eqn7 = j(Plan(2,1)) == 0;
eqn8 = j(Plan(3,1)) == 0;
eqns = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6 eqn7 eqn8];

% Solver
S = solve(eqns);

x2(t)=subs(x(t),c0,double(S.c0));
x2(t)=subs(x2(t),c1,double(S.c1));
x2(t)=subs(x2(t),c2,double(S.c2));
x2(t)=subs(x2(t),c3,double(S.c3));
x2(t)=subs(x2(t),c4,double(S.c4));
x2(t)=subs(x2(t),c5,double(S.c5));
x2(t)=subs(x2(t),c6,double(S.c6));
x2(t)=subs(x2(t),c7,double(S.c7));


v2(t)=subs(v(t),c0,double(S.c0));
v2(t)=subs(v2(t),c1,double(S.c1));
v2(t)=subs(v2(t),c2,double(S.c2));
v2(t)=subs(v2(t),c3,double(S.c3));
v2(t)=subs(v2(t),c4,double(S.c4));
v2(t)=subs(v2(t),c5,double(S.c5));
v2(t)=subs(v2(t),c6,double(S.c6));
v2(t)=subs(v2(t),c7,double(S.c7));


a2(t)=subs(a(t),c0,double(S.c0));
a2(t)=subs(a2(t),c1,double(S.c1));
a2(t)=subs(a2(t),c2,double(S.c2));
a2(t)=subs(a2(t),c3,double(S.c3));
a2(t)=subs(a2(t),c4,double(S.c4));
a2(t)=subs(a2(t),c5,double(S.c5));
a2(t)=subs(a2(t),c6,double(S.c6));
a2(t)=subs(a2(t),c7,double(S.c7));

j2(t)=subs(j(t),c0,double(S.c0));
j2(t)=subs(j2(t),c1,double(S.c1));
j2(t)=subs(j2(t),c2,double(S.c2));
j2(t)=subs(j2(t),c3,double(S.c3));
j2(t)=subs(j2(t),c4,double(S.c4));
j2(t)=subs(j2(t),c5,double(S.c5));
j2(t)=subs(j2(t),c6,double(S.c6));
j2(t)=subs(j2(t),c7,double(S.c7));

% P3 to P4 
% Minimum Snap Equations
x(t) = c7*(t-Plan(3,1))^7 + c6*(t-Plan(3,1))^6 + c5*(t-Plan(3,1))^5+c4*(t-Plan(3,1))^4+c3*(t-Plan(3,1))^3+c2*(t-Plan(3,1))^2+c1*(t-Plan(3,1))+c0;
v(t) = 7*c7*(t-Plan(3,1))^6 + 6*c6*(t-Plan(3,1))^5 + 5*c5*(t-Plan(3,1))^4 + 4*c4*(t-Plan(3,1))^3 + 3*c3*(t-Plan(3,1))^2 + 2*c2*(t-Plan(3,1)) + c1
a(t) = 42*c7*(t-Plan(3,1))^5 + 30*c6*(t-Plan(3,1))^4 + 20*c5*(t-Plan(3,1))^3 + 12*c4*(t-Plan(3,1))^2 +6*c3*(t-Plan(3,1)) + 2*c2
j(t) = 210*c7*(t-Plan(3,1))^4 + 120*c6*(t-Plan(3,1))^3 + 60*c5*(t-Plan(3,1))^2 + 24*c4*(t-Plan(3,1)) + 6*c3 ;

% Boundary Conditions
eqn1 = x(Plan(3,1)) == Plan(3,2);
eqn2 = x(Plan(4,1)) == Plan(4,2);
eqn3 = a(Plan(3,1)) == 0;
eqn4 = a(Plan(4,1)) == 0;
eqn5 = v(Plan(3,1)) == V_Road;
eqn6 = v(Plan(4,1)) == V_Road;
eqn7 = j(Plan(3,1)) == 0;
eqn8 = j(Plan(4,1)) == 0;
eqns = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6 eqn7 eqn8];

% Solver
S = solve(eqns);

x3(t)=subs(x(t),c0,double(S.c0));
x3(t)=subs(x3(t),c1,double(S.c1));
x3(t)=subs(x3(t),c2,double(S.c2));
x3(t)=subs(x3(t),c3,double(S.c3));
x3(t)=subs(x3(t),c4,double(S.c4));
x3(t)=subs(x3(t),c5,double(S.c5));
x3(t)=subs(x3(t),c6,double(S.c6));
x3(t)=subs(x3(t),c7,double(S.c7));


v3(t)=subs(v(t),c0,double(S.c0));
v3(t)=subs(v3(t),c1,double(S.c1));
v3(t)=subs(v3(t),c2,double(S.c2));
v3(t)=subs(v3(t),c3,double(S.c3));
v3(t)=subs(v3(t),c4,double(S.c4));
v3(t)=subs(v3(t),c5,double(S.c5));
v3(t)=subs(v3(t),c6,double(S.c6));
v3(t)=subs(v3(t),c7,double(S.c7));

a3(t)=subs(a(t),c0,double(S.c0));
a3(t)=subs(a3(t),c1,double(S.c1));
a3(t)=subs(a3(t),c2,double(S.c2));
a3(t)=subs(a3(t),c3,double(S.c3));
a3(t)=subs(a3(t),c4,double(S.c4));
a3(t)=subs(a3(t),c5,double(S.c5));
a3(t)=subs(a3(t),c6,double(S.c6));
a3(t)=subs(a3(t),c7,double(S.c7));

j3(t)=subs(j(t),c0,double(S.c0));
j3(t)=subs(j3(t),c1,double(S.c1));
j3(t)=subs(j3(t),c2,double(S.c2));
j3(t)=subs(j3(t),c3,double(S.c3));
j3(t)=subs(j3(t),c4,double(S.c4));
j3(t)=subs(j3(t),c5,double(S.c5));
j3(t)=subs(j3(t),c6,double(S.c6));
j3(t)=subs(j3(t),c7,double(S.c7));

% P4 to P5 
% Minimum Snap Equations
x(t) = c7*(t-Plan(4,1))^7 + c6*(t-Plan(4,1))^6 + c5*(t-Plan(4,1))^5+c4*(t-Plan(4,1))^4+c3*(t-Plan(4,1))^3+c2*(t-Plan(4,1))^2+c1*(t-Plan(4,1))+c0;
v(t) = 7*c7*(t-Plan(4,1))^6 + 6*c6*(t-Plan(4,1))^5 + 5*c5*(t-Plan(4,1))^4 + 4*c4*(t-Plan(4,1))^3 + 3*c3*(t-Plan(4,1))^2 + 2*c2*(t-Plan(4,1)) + c1
a(t) = 42*c7*(t-Plan(4,1))^5 + 30*c6*(t-Plan(4,1))^4 + 20*c5*(t-Plan(4,1))^3 + 12*c4*(t-Plan(4,1))^2 +6*c3*(t-Plan(4,1)) + 2*c2
j(t) = 210*c7*(t-Plan(4,1))^4 + 120*c6*(t-Plan(4,1))^3 + 60*c5*(t-Plan(4,1))^2 + 24*c4*(t-Plan(4,1)) + 6*c3 ;

% Boundary Conditions
eqn1 = x(Plan(4,1)) == Plan(4,2);
eqn2 = x(Plan(5,1)) == Plan(5,2);
eqn3 = a(Plan(4,1)) == 0;
eqn4 = a(Plan(5,1)) == 0;
eqn5 = v(Plan(4,1)) == V_Road;
eqn6 = v(Plan(5,1)) == V_Road;
eqn7 = j(Plan(4,1)) == 0;
eqn8 = j(Plan(5,1)) == 0;
eqns = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6 eqn7 eqn8];

% Solver
S = solve(eqns);

x4(t)=subs(x(t),c0,double(S.c0));
x4(t)=subs(x4(t),c1,double(S.c1));
x4(t)=subs(x4(t),c2,double(S.c2));
x4(t)=subs(x4(t),c3,double(S.c3));
x4(t)=subs(x4(t),c4,double(S.c4));
x4(t)=subs(x4(t),c5,double(S.c5))
x4(t)=subs(x4(t),c6,double(S.c6));
x4(t)=subs(x4(t),c7,double(S.c7));

v4(t)=subs(v(t),c0,double(S.c0));
v4(t)=subs(v4(t),c1,double(S.c1));
v4(t)=subs(v4(t),c2,double(S.c2));
v4(t)=subs(v4(t),c3,double(S.c3));
v4(t)=subs(v4(t),c4,double(S.c4));
v4(t)=subs(v4(t),c5,double(S.c5));
v4(t)=subs(v4(t),c6,double(S.c6));
v4(t)=subs(v4(t),c7,double(S.c7));

a4(t)=subs(a(t),c0,double(S.c0));
a4(t)=subs(a4(t),c1,double(S.c1));
a4(t)=subs(a4(t),c2,double(S.c2));
a4(t)=subs(a4(t),c3,double(S.c3));
a4(t)=subs(a4(t),c4,double(S.c4));
a4(t)=subs(a4(t),c5,double(S.c5));
a4(t)=subs(a4(t),c6,double(S.c6));
a4(t)=subs(a4(t),c7,double(S.c7));

j4(t)=subs(j(t),c0,double(S.c0));
j4(t)=subs(j4(t),c1,double(S.c1));
j4(t)=subs(j4(t),c2,double(S.c2));
j4(t)=subs(j4(t),c3,double(S.c3));
j4(t)=subs(j4(t),c4,double(S.c4));
j4(t)=subs(j4(t),c5,double(S.c5));
j4(t)=subs(j4(t),c6,double(S.c6));
j4(t)=subs(j4(t),c7,double(S.c7));

% P5 to P6 
% Minimum Snap Equations
x(t) = c7*(t-Plan(5,1))^7 + c6*(t-Plan(5,1))^6 + c5*(t-Plan(5,1))^5+c4*(t-Plan(5,1))^4+c3*(t-Plan(5,1))^3+c2*(t-Plan(5,1))^2+c1*(t-Plan(5,1))+c0;
v(t) = 7*c7*(t-Plan(5,1))^6 + 6*c6*(t-Plan(5,1))^5 + 5*c5*(t-Plan(5,1))^4 + 4*c4*(t-Plan(5,1))^3 + 3*c3*(t-Plan(5,1))^2 + 2*c2*(t-Plan(5,1)) + c1
a(t) = 42*c7*(t-Plan(5,1))^5 + 30*c6*(t-Plan(5,1))^4 + 20*c5*(t-Plan(5,1))^3 + 12*c4*(t-Plan(5,1))^2 +6*c3*(t-Plan(5,1)) + 2*c2
j(t) = 210*c7*(t-Plan(5,1))^4 + 120*c6*(t-Plan(5,1))^3 + 60*c5*(t-Plan(5,1))^2 + 24*c4*(t-Plan(5,1)) + 6*c3 ;

%Boundary Conditions
eqn1 = x(Plan(5,1)) == Plan(5,2);
eqn2 = x(Plan(6,1)) == Plan(6,2);
eqn3 = a(Plan(5,1)) == 0;
eqn4 = a(Plan(6,1)) == 0;
eqn5 = v(Plan(5,1)) == V_Road;
eqn6 = v(Plan(6,1)) == 0;
eqn7 = j(Plan(5,1)) == 0;
eqn8 = j(Plan(6,1)) == 0;
eqns = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6 eqn7 eqn8];

% Solver
S = solve(eqns);

x5(t)=subs(x(t),c0,double(S.c0));
x5(t)=subs(x5(t),c1,double(S.c1));
x5(t)=subs(x5(t),c2,double(S.c2));
x5(t)=subs(x5(t),c3,double(S.c3));
x5(t)=subs(x5(t),c4,double(S.c4));
x5(t)=subs(x5(t),c5,double(S.c5));
x5(t)=subs(x5(t),c6,double(S.c6));
x5(t)=subs(x5(t),c7,double(S.c7));

v5(t)=subs(v(t),c0,double(S.c0));
v5(t)=subs(v5(t),c1,double(S.c1));
v5(t)=subs(v5(t),c2,double(S.c2));
v5(t)=subs(v5(t),c3,double(S.c3));
v5(t)=subs(v5(t),c4,double(S.c4));
v5(t)=subs(v5(t),c5,double(S.c5));
v5(t)=subs(v5(t),c6,double(S.c6));
v5(t)=subs(v5(t),c7,double(S.c7));

a5(t)=subs(a(t),c0,double(S.c0));
a5(t)=subs(a5(t),c1,double(S.c1));
a5(t)=subs(a5(t),c2,double(S.c2));
a5(t)=subs(a5(t),c3,double(S.c3));
a5(t)=subs(a5(t),c4,double(S.c4));
a5(t)=subs(a5(t),c5,double(S.c5));
a5(t)=subs(a5(t),c6,double(S.c6));
a5(t)=subs(a5(t),c7,double(S.c7));

j5(t)=subs(j(t),c0,double(S.c0));
j5(t)=subs(j5(t),c1,double(S.c1));
j5(t)=subs(j5(t),c2,double(S.c2));
j5(t)=subs(j5(t),c3,double(S.c3));
j5(t)=subs(j5(t),c4,double(S.c4));
j5(t)=subs(j5(t),c5,double(S.c5));
j5(t)=subs(j5(t),c6,double(S.c6));
j5(t)=subs(j5(t),c7,double(S.c7));



% Time vector 
t_plot =0:0.0001:Plan(end,1);


% Position Plot
figure(1)
% P1-P2
ind_start = find(t_plot==Plan(1,1));
ind_end = find(t_plot==Plan(2,1));
x1c = double(x1(t_plot));
plot(t_plot(ind_start:ind_end),x1c(ind_start:ind_end),'Linewidth',1)
hold on 

% P2-P3
ind_start = find(t_plot==Plan(2,1));
ind_end = find(t_plot==Plan(3,1));
x2c = double(x2(t_plot));
plot(t_plot(ind_start:ind_end),x2c(ind_start:ind_end),'Linewidth',1)

% P3-P4
ind_start = find(t_plot==Plan(3,1));
ind_end = find(t_plot==Plan(4,1));
x3c = double(x3(t_plot));
plot(t_plot(ind_start:ind_end),x3c(ind_start:ind_end),'Linewidth',1)

% P4-P5
ind_start = find(t_plot==Plan(4,1));
ind_end = find(t_plot==Plan(5,1));
x4c = double(x4(t_plot));
plot(t_plot(ind_start:ind_end),x4c(ind_start:ind_end),'Linewidth',1)

% P5-P6
ind_start = find(t_plot==Plan(5,1));
ind_end = find(t_plot==Plan(6,1));
x5c = double(x5(t_plot));
plot(t_plot(ind_start:ind_end),x5c(ind_start:ind_end),'Linewidth',1)

% Desired Points
plot(0,0,'Marker',"+",'Color',"r");
plot(0.1452,0.2903,'Marker',"+",'Color',"r");
plot(0.3210,0.6420,'Marker',"+",'Color',"r");
plot(0.4919,0.9837,'Marker',"+",'Color',"r");
plot(0.6211,1.2429,'Marker',"+",'Color',"r");
plot(0.779,1.5558,'Marker',"+",'Color',"r");

% For printing purpose
% width=15;
% height=10;
% set(gcf,'units','centimeters','position',[2,2,width,height])
% grid on;
% xlabel('Time [s]')
% ylabel('Position [m]')
% 
% ax = gcf;
% exportgraphics(ax,'SnapPos.png','Resolution',300)



% Velocity Plot
% P1-P2
figure(3)
ind_start = find(t_plot==Plan(1,1));
ind_end = find(t_plot==Plan(2,1));
v1c = double(v1(t_plot));
plot(t_plot(ind_start:ind_end),v1c(ind_start:ind_end),'Linewidth',1)
hold on 

% P2-P3
ind_start = find(t_plot==Plan(2,1));
ind_end = find(t_plot==Plan(3,1));
v2c = double(v2(t_plot));
plot(t_plot(ind_start:ind_end),v2c(ind_start:ind_end),'Linewidth',1)

% P3-P4
ind_start = find(t_plot==Plan(3,1));
ind_end = find(t_plot==Plan(4,1));
v3c = double(v3(t_plot));
plot(t_plot(ind_start:ind_end),v3c(ind_start:ind_end),'Linewidth',1)

% P4-P5
ind_start = find(t_plot==Plan(4,1));
ind_end = find(t_plot==Plan(5,1));
v4c = double(v4(t_plot));
plot(t_plot(ind_start:ind_end),v4c(ind_start:ind_end),'Linewidth',1)

% P5-P6
ind_start = find(t_plot==Plan(5,1));
ind_end = find(t_plot==Plan(6,1));
v5c = double(v5(t_plot));
plot(t_plot(ind_start:ind_end),v5c(ind_start:ind_end),'Linewidth',1)

% For printig purpose
% width=15;
% height=10;
% set(gcf,'units','centimeters','position',[2,2,width,height])
% grid on;
% xlabel('Time [s]')
% ylabel('Velocity [m/s]')
% 
% ax = gcf;
% exportgraphics(ax,'SnapVel.png','Resolution',300)


% Acceleration Plot
figure(4)
% P1-P2
ind_start = find(t_plot==Plan(1,1));
ind_end = find(t_plot==Plan(2,1));
a1c = double(a1(t_plot));
plot(t_plot(ind_start:ind_end),a1c(ind_start:ind_end),'Linewidth',1)
hold on 

% P2-P3
ind_start = find(t_plot==Plan(2,1));
ind_end = find(t_plot==Plan(3,1));
a2c = double(a2(t_plot));
plot(t_plot(ind_start:ind_end),a2c(ind_start:ind_end),'Linewidth',1)

% P3-P4
ind_start = find(t_plot==Plan(3,1));
ind_end = find(t_plot==Plan(4,1));
a3c = double(a3(t_plot));
plot(t_plot(ind_start:ind_end),a3c(ind_start:ind_end),'Linewidth',1)

% P4-P5
ind_start = find(t_plot==Plan(4,1));
ind_end = find(t_plot==Plan(5,1));
a4c = double(a4(t_plot));
plot(t_plot(ind_start:ind_end),a4c(ind_start:ind_end),'Linewidth',1)

% P5-P6
ind_start = find(t_plot==Plan(5,1));
ind_end = find(t_plot==Plan(6,1));
a5c = double(a5(t_plot));
plot(t_plot(ind_start:ind_end),a5c(ind_start:ind_end),'Linewidth',1)

% For printing purposes
% width=15;
% height=10;
% set(gcf,'units','centimeters','position',[2,2,width,height])
% grid on;
% xlabel('Time [s]')
% ylabel('Acceleration [m/s^2]')
% 
% ax = gcf;
% exportgraphics(ax,'SnapAcc.png','Resolution',300)

% Jerk Plot
figure(5)
% P1-P2
ind_start = find(t_plot==Plan(1,1));
ind_end = find(t_plot==Plan(2,1));
j1c = double(j1(t_plot));
plot(t_plot(ind_start:ind_end),j1c(ind_start:ind_end),'Linewidth',1)
hold on 

% P2-P3
ind_start = find(t_plot==Plan(2,1));
ind_end = find(t_plot==Plan(3,1));
j2c = double(j2(t_plot));
plot(t_plot(ind_start:ind_end),j2c(ind_start:ind_end),'Linewidth',1)

% P3-P4
ind_start = find(t_plot==Plan(3,1));
ind_end = find(t_plot==Plan(4,1));
j3c = double(j3(t_plot));
plot(t_plot(ind_start:ind_end),j3c(ind_start:ind_end),'Linewidth',1)

% P4-P5
ind_start = find(t_plot==Plan(4,1));
ind_end = find(t_plot==Plan(5,1));
j4c = double(j4(t_plot));
plot(t_plot(ind_start:ind_end),j4c(ind_start:ind_end),'Linewidth',1)

% P5-P6
ind_start = find(t_plot==Plan(5,1));
ind_end = find(t_plot==Plan(6,1));
j5c = double(j5(t_plot));
plot(t_plot(ind_start:ind_end),j5c(ind_start:ind_end),'Linewidth',1)

% For printing purposes
% width=15;
% height=10;
% set(gcf,'units','centimeters','position',[2,2,width,height])
% grid on;
% xlabel('Time [s]')
% ylabel('Jerk [m/s^3]')
% 
% ax = gcf;
% exportgraphics(ax,'SnapJerk.png','Resolution',300)





