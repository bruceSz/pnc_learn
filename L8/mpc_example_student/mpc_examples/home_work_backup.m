clear all;
close all;
clc;

p_0 = [0 8 20];
v_0 = [0 0 0];
a_0 = [0 0 0];
K=20;
dt=0.2;

P=[];
V=[];
A=[];


%% Construct the reference signal

% compute in advance the reference signal
t = 0.2;
for i = 1:200
    tref = t + i*0.2;
    
    r=0.25*tref;
    pt(i,1) = r*sin(0.5*tref);
    vt(i,1) = r*cos(0.5*tref);
    at(i,1) = -r*sin(0.5*tref);
    
    pt(i,2) = r*cos(0.5*tref);
    vt(i,2) = -r*sin(0.5*tref);
    at(i,2) = -r*cos(0.5*tref);
    
    pt(i,3) = 20 - 0.5*tref;
    vt(i,3) = -0.5;
    at(i,3) = 0;
    disp("tref is: " + tref + " z is: " + pt(i, 3));
    if pt(i, 3) < 0
        disp("z is : "  + pt(i,3));
        break;
    end

end

max_idx = size(pt,1)


P = [p_0];
V = [v_0];
Acc = [a_0];

w1 = 100;
w2 = 1;
w3 = 1;
w4 = 1;
w5 = 1e4;

p_0 = [0 8 20];
v_0 = [0 0 0];
a_0 = [0 0 0];

for t=1:40
    
    % here the t is the index,
    
    % for each t-interval, getPredictionMatrix for each axis.
    % here the Bp* is update to (Bp* - pt)
    % here the Bv* is update to (Bv* - vt)
    % here the Ba* is update to (Ba* - at)
    pt_t = pt(t,:)
    pt_t1 = pt_t(1)
    pt_t2 = pt_t(2)
    pt_t3 = pt_t(3)

    vt_t = vt(t,:)
    vt_t1 = vt_t(1)
    vt_t2 = vt_t(2)
    vt_t3 = vt_t(3)

    at_t = at(t,:)
    at_t1 = at_t(1)
    at_t2 = at_t(2)
    at_t3 = at_t(3)

    
    [Tp1, Tv1, Ta1, Bp1, Bv1, Ba1] = getPredictionMatrix(K,dt,p_0(1),v_0(1),a_0(1));
    [Tp2, Tv2, Ta2, Bp2, Bv2, Ba2] = getPredictionMatrix(K,dt,p_0(2),v_0(2),a_0(2));
    [Tp3, Tv3, Ta3, Bp3, Bv3, Ba3] = getPredictionMatrix(K,dt,p_0(3),v_0(3),a_0(3));

    % for each axis, compute the optimal
    %% Construct the optimization problem
    
    % axis1

    Bp1 = Bp1 - pt_t1;
    Bv1 = Bv1 - vt_t1;
    Ba1 = Ba1 - at_t1;
    H = blkdiag(w4*eye(K)+w1*(Tp1'*Tp1)+w2*(Tv1'*Tv1)+w3*(Ta1'*Ta1),w5*eye(K));
    F = [w1*(Bp1)'*Tp1+w2*(Bv1)'*Tv1+w3*(Ba1)'*Ta1 zeros(1,K)];
    %disp("size of tv1:"  + size(Tv1));
    %disp("size of ta1:"  + size(Ta1));
    %disp("sizeof eye k: " + size(eye(K)));
    
    %A = [Tv1;-Tv1;Ta1;-Ta1;eye(K);-eye(K)];
    %b = [ones(K,1)*6-Bv1;ones(K,1)*6+Bv1;ones(K,1)*3-Ba1;ones(K,1)*3+Ba1; ones(K,1)* 3;ones(K,1)*3;zeros(K,1)];
    
    A = [Tv1 zeros(K);-Tv1 -eye(K);Ta1 zeros(K); -Ta1 zeros(K); zeros(size(Ta1)) -eye(K); 
        eye(K) zeros(K); -eye(K) zeros(K)];
    b = [ones(20,1)-Bv1;ones(20,1)+Bv1;ones(20,1)-Ba1;ones(20,1)+Ba1; zeros(K,1);
        ones(K,1)* 3;ones(K,1)*3; ];    

    %% Solve the optimization problem
    J = quadprog(H,F,A,b);
    
    disp("t is : " + t)
    %% Apply the control
    j(1) = J(1);
    %%%%%%%%%%%%%%%%


    % axis2
    %% Construct the optimization problem

    Bp2 = Bp2 - pt_t2;
    Bv2 = Bv2 - vt_t2;
    Ba2 = Ba2 - at_t2;
    H = w4*eye(K)+w1*(Tp2'*Tp2)+w2*(Tv2'*Tv2)+w3*(Ta2'*Ta2);
    F = w1*(Bp2)'*Tp1+w2*(Bv2)'*Tv2+w3*(Ba2)'*Ta2;
    A = [Tv2;-Tv2;Ta2;-Ta2;eye(K);-eye(K)];
    b = [ones(K,1)*6-Bv2;ones(K,1)*6+Bv2;ones(K,1)*3-Ba2;ones(K,1)*3+Ba2; ones(K,1)* 3;ones(K,1)*3];

    %% Solve the optimization problem
    J = quadprog(H,F,A,b);

    %% Apply the control
    j(2) = J(1);
    %%%%%%%%%%%%%%%%%


    % axis3
    %% Construct the optimization problem
    
    Bp3 = Bp3 - pt_t3;
    Bv3 = Bv3 - vt_t3;
    Ba3 = Ba3 - at_t3;
    H = w4*eye(K)+w1*(Tp3'*Tp3)+w2*(Tv3'*Tv3)+w3*(Ta3'*Ta3);
    F = w1*(Bp3)'*Tp3+w2*(Bv3)'*Tv1+w3*(Ba3)'*Ta3;
    A = [Tv3;-Tv3;Ta3;-Ta3;eye(K);-eye(K)];
    b = [ones(K,1)*6-Bv1;ones(K,1)+Bv1;ones(K,1)*3-Ba1;ones(K,1)+Ba1; ones(K,1)* 2;ones(K,1)*2];
    %% Solve the optimization problem
    J = quadprog(H,F,A,b);
    
    %% Apply the control
    j(3) = J(1);

    %%%%%%%%%%%%%%%%%%


    %% Do the MPC
    %% Please follow the example in linear mpc part to fill in the code here to do the tracking
    

    % compute new p_0, v_0, a_0 .
    for i=1:3
       [p_0(i),v_0(i),a_0(i)] = forward(p_0(i),v_0(i),a_0(i),j(i),dt);
    end

    %if t > max_idx
    %    disp("size of pt is: " + t)
    %    break
    %end
    
    %% Log the states
    disp("size of P" + size(P));
    disp("size of A:" + size(A));
    disp("size of V" + size(V));
    disp("size of a0:" + size(a_0));

    P = [P;p_0 ];
    V = [V;v_0 ];
    Acc = [Acc;a_0 ];
end
disp("p0 is : " + p_0)
disp( "pt at t: " + pt(t,:))
disp( "vt at t: " + vt(t,:))
disp( "at at t: " + vt(t,:))

disp(" t is: " + t)

disp("max idx is: " + max_idx)

plot3(P(:,1),P(:,2),P(:,3));
hold on;
plot3(pt(:,1),pt(:,2),pt(:,3))
hold on;
%% Plot the result
%plot(P);
%grid on;
%legend('x','y','z');
%figure;
%plot3(P(:,1),P(:,2),P(:,3));
%axis equal;
%grid on;