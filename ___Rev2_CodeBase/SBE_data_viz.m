%% SBE_data_viz

clear; clc; close all

% rc sailing data
% data = csvread("data/7-19-sail-rc/Moth_Data_2000-1-1_0h12m5s.csv");
% data = csvread("data/7-19-sail-rc/Moth_Data_2000-1-1_0h16m16s.csv");
% data = csvread("data/7-19-sail-rc/Moth_Data_2000-1-1_0h26m4s.csv");
% data = csvread("data/7-19-sail-rc/Moth_Data_2000-1-1_0h33m42s.csv");

% day one sailing data
% data = csvread("data/7-19-sail-wand/Moth_Data_2000-1-1_0h31m45s.csv"); % good 
% data = csvread("data/7-19-sail-wand/Moth_Data_2000-1-1_0h42m15s.csv");
% data = csvread("data/7-19-sail-wand/Moth_Data_2000-1-1_0h47m7s.csv");
% data = csvread("data/7-19-sail-wand/Moth_Data_2000-1-1_0h52m56s.csv"); % good set
% data = csvread("data/7-19-sail-wand/Moth_Data_2000-1-1_1h0m20s.csv"); % good set
% data = csvread("data/7-19-sail-wand/Moth_Data_2000-1-1_1h6m9s.csv");

% day two sailing data
% data = csvread("data/7-20/Moth_Data_2000-1-1_0h55m39s.csv"); % run 1
% data = csvread("data/7-20/Moth_Data_2000-1-1_0h57m23s.csv"); % run 2 
% data = csvread("data/7-20/Moth_Data_2000-1-1_0h58m54s.csv"); % run 3 
% data = csvread("data/7-20/Moth_Data_2000-1-1_1h1m19s.csv"); % run 4

% day three, successful auto sailing!
data = csvread("data/7-21/Moth_Data_2000-1-1_0h48m11s.csv"); 

data =      data(5:end,:); % truncate initial data

% micros =    data(:,1);
% poten =     data(:,2);
% US =        data(:,3);
% GPS_spd =   data(:,4);
% Heave_acc = data(:,5);
% Heel_ang =  data(:,6);
% Pitch_ang = data(:,7);

micros =    data(:,1);
poten =     data(:,2);
ctl_in_sig = data(:,3);
flap_perc = data(:,4);
US =        data(:,5);
GPS_spd =   data(:,6);
Heave_acc = data(:,7);
Heel_ang =  data(:,8);
Pitch_ang = data(:,9);

%%
% attempt to integrate the heave...
heave_vel(1) = 0;
rh(1) = 0;

t(1) = 0;

filt_heave_acc(1) = Heave_acc(1);
counts = 1;
for k = 1:length(data(:,1))-1
    weight = 0.2;
    filt_heave_acc(k+1) = filt_heave_acc(k)*(1-weight) + Heave_acc(k+1)*weight;
    
    
    if micros(k+1) > micros(k)
        delta_t(k) = (micros(k+1)-micros(k))/1000;
    else
        delta_t(k) = (micros(k+1) + 1000000 - micros(k))/1000;
    end
    delta_t(k) = delta_t(k)/1000; % seconds rather than ms
    
    heave_vel(k+1) = heave_vel(k) + Heave_acc(k) * delta_t(k);
    rh(k+1) = rh(k) + heave_vel(k)*delta_t(k) + 1/2*Heave_acc(k)*delta_t(k)^2;
    
    
    t(k+1) = t(k)+delta_t(k);
    
    if abs(US(k+1)-US(k)) > 300
        flags(counts) = k+1;
        counts = counts + 1;
    end

end

%%
i = 1;
for k = 1:length(US)-1
    if 10 < t(k) && t(k) < 54.5
        t(k);
        delUS(i) = US(k+1)-US(k);
        i = i+1;
    end
end
std(delUS)
% figure()
% histogram(delUS)
%%
figure()

subplot(4,1,1)
plot(t,poten);
grid on

subplot(4,1,2)
plot(t,GPS_spd)
grid on

subplot(4,1,3)
plot(t,US,'b-o')
grid on
ylim([0, 2000])

subplot(4,1,4)
plot(t,Heave_acc)
grid on
hold on 
plot(t, filt_heave_acc,'r')

figure()
plot(t,Heave_acc)
hold on
plot(t,heave_vel)
plot(t,rh)
legend('Acceleration','m/s','m')
grid on
% 

%%

RH_init = 500; % initial rh guess 
RH(1) = US(1);
RH_v_est(1) = 0; % init velocity derivative estimate to be 0
counter = 0;

RHB(1) = US(1);

for k = 1:length(data(:,1))-1
    % outlier rejection experimentation
     if US(k+1) < 10 % throw out dropped (too close) data points
            RHB(k+1) = RHB(k);
        elseif (US(k+1) - RHB(k)) > 300 % can have a different delta thresh for up and down
            RHB(k+1) = RHB(k);
        elseif (RHB(k) - US(k+1)) > 900 % can have a different delta thresh for up and down
            RHB(k+1) = RHB(k);
        else
            RHB(k+1) = US(k+1);
     end
        
        if (US(k+1) < 10) && (counter < 3) % throw out dropped (too close) data points
            if (500 < RH(k) < 1300)
                RH(k+1) = RH(k);
            else 
                RH(k+ 1) = 1000;
            end
            counter = counter + 1;
        elseif ((US(k+1) - RH(k)) > 300) && (counter < 3) % can have a different delta thresh for up and down
            if (500 < RH(k) < 1300)
                RH(k+1) = RH(k);
            else 
                RH(k+ 1) = 1000;
            end
            counter = counter + 1;
        elseif ((RH(k) - US(k+1)) > 900) && (counter < 3) % can have a different delta thresh for up and down
             if (500 < RH(k) < 1300)
                RH(k+1) = RH(k);
            else 
                RH(k+ 1) = 1000;
            end
            counter = counter + 1;
        else
            RH(k+1) = US(k+1);
            counter = 0;
        end
        
%         RH_v_est(k+1) = ((RH(k+1)-RH(k))/1000)/delta_t(k); % bit jank
        RH_v_est(k+1) = RH_v_est(k)*.99 + 0.01*((RH(k+1)-RH(k))/1000)/delta_t(k); % bit jank
end

figure()
subplot(2,1,1)
plot(t,US,'b-o')
grid on
hold on
% plot(t(flags),US(flags),'ro','markerfacecolor','r')
plot(t,RHB,'m','linewidth',1)
plot(t,RH,'g-','linewidth',1)
ylim([0,2000])

subplot(2,1,2)
plot(t,Heel_ang,'r')
hold on 
plot(t,Pitch_ang,'g')
grid on
legend('heel','pitch')

% start = 6667;
% stop = 6788;
% 
% vel_est = zeros(length(t));
% for jj = start:stop
%    vel_est(jj+1) = vel_est(jj) + delta_t(jj)*Heave_acc(jj);
% end
% figure()
% plot(t(start:stop),Heave_acc(start:stop),'bo-')
% grid on
% hold on 
% plot(t(start:stop),vel_est(start:stop),'ro-')


%% KF 

% discrete time state transition matrix
F = zeros(3); % will update shortly

n = size(F,1);

% H measurement matrix
% H = [1,0,0;
%      0,1,0;
%      0,0,1];
H = [1,0,0;
     0,0,1];
 
% process noise definition
% Omega = dt*[0,0,0;1,0,0;0,0,0;0,1,0;0,0,0;0,0,1];
% Q = diag([5,5,5]);
% Qkf = Omega*Q*Omega';
Qkf = [.100, 0, 0;
       0, 0.01, 0;
       0, 0, 0.1];

% measurement noise definition 
% R = [.020,0,0;
%      0,0.100,0;
%      0,0,1.5];  
R = [.020,0;
     0,0.1]; 
 
% x = rh, rh vel, rh acc

P(:,:,1) = 1*diag([0.200,0.2,0.02]);

% rng(100)
% x(:,1) = mvnrnd(mean_init,P(:,:,1))
% x = RH, RHdot, RHddot
x(:,1) = [0.500; 0.0; 0.0];
for k = 1:length(US(:,1))-1 
    
    % gather measurement vector
%     y(:,k) = [RH(k)/1000; RH_v_est(k); Heave_acc(k)]; 
    y(:,k) = [RH(k)/1000; Heave_acc(k)];
    % recompute F since it is time delta dependent 
    F = [1, delta_t(k), 0.5*delta_t(k)^2;
        0, 1, delta_t(k);
        0, 0, 1];
    
    %%Perform prediction step
    x_minus = F*x(:,k);
    P_minus = F*P(:,:,k)*transpose(F) + Qkf;
    
    %%Compute Kalman gain
    K = P_minus*transpose(H)/(H*P_minus*transpose(H)+ R);
    
    %%Perform measurement update step
    x(:,k+1) = x_minus + K*(y(k) - H*x_minus);
    P(:,:,k+1) = (eye(n) - K*H)*P_minus;
end
% figure()
% plot(syn_time(:,1),x(1,:),'r')
% hold on
% plot(syn_time(:,1),x(3,:),'g')
% plot(syn_time(:,1),x(5,:),'b')
% title('Synthetic IMU KF pose estimates')
% xlabel('time')
% ylabel('degrees')
% legend('roll','pitch','yaw')

figure()
% figure(2)
% hold on
plot(t,x(1,:),'g')
hold on
plot(t,x(2,:),'r')
plot(t,x(3,:),'k')
plot(t,RH_v_est,'m')
legend('KF rh','KF rh v','IMU acc','RH_v_est')
grid on

%%
n_poten = normalize(poten);
ctrl_sig(1) = RH(1);
wegt = 0.01;

P = 0.005;
I = 0.001;
D = 0.0;
setpt = 1250;
cmd(1) = 0.5;
% err_hist = 
% derr = 

for k = 1:length(data(:,1))-1
    ctrl_sig(k+1) = (1-wegt)*ctrl_sig(k) + wegt*RH(k+1);
    
    err = setpt - ctrl_sig(k+1);
    cmd(k+1) = P*err;
end

figure()
[AX H1 H2] = plotyy(t,ctrl_sig,t,n_poten);
grid on
hold(AX(2));
plot(AX(2),t,cmd,'m');
set(AX(1),'YLim',[300 1800])

set(AX(1),'Box','off')
set(AX(1),'YTick',[300:100:1800])

% figure(3)
% hold on
% plot(t,ctrl_sig,'m-')

figure()
plot(t,normalize(1-poten))
hold on
plot(t,normalize(ctrl_sig))
legend('poten','us_ctrl_sig')

%% 
% close all
% clc

setpt = 1100;
P = 0.001;
err(1) = 0;
cmd(1) = 0;

t(1) = 0;
for k = 1:length(data(:,1))-1

    if micros(k+1) > micros(k)
        delta_t(k) = (micros(k+1)-micros(k))/1000;
    else
        delta_t(k) = (micros(k+1) + 1000000 - micros(k))/1000;
    end
    delta_t(k) = delta_t(k)/1000; % seconds rather than ms
    t(k+1) = t(k)+delta_t(k); % get time 
    
    % outlier rejection experimentation
    
    RH_init = 500; % initial rh guess 
    RH(1) = RH_init;    
    if US(k+1) < 10 % throw out dropped (too close) data points
        RH(k+1) = RH(k);
    elseif (US(k+1) - RH(k)) > 300 % can have a different delta thresh for up and down
        RH(k+1) = RH(k);
    elseif (RH(k) - US(k+1)) > 900 % can have a different delta thresh for up and down
        RH(k+1) = RH(k);
    else
        RH(k+1) = US(k+1);
    end
    
    err(k+1) = setpt - RH(k+1);
    cmd(k+1) = P*err(k+1);
    if cmd(k+1) > 1
        cmd(k+1) = 1;
    elseif cmd(k+1) < 0
        cmd(k+1) = 0;
    end
end


starti = 60;
ind = find(t>starti);
figure()

subplot(2,1,1)
plot(t,1-poten,'-ob')
ylim([0,1]);
grid on;
hold on 
plot(t,cmd,'-r')

subplot(2,1,2)
plot(t,RH,'-o')
ylim([450,1600]);
grid on;
%    
% for k = ind(1):length(data(:,1))-1-100
%    cla;
%    subplot(2,1,1)
%    plot(t(k:k+80),1-poten(k:k+80),'-ob')
%    ylim([0,1]);
%    xlim([t(k),t(k+95)]);
%    grid on;
%    hold on 
%    plot(t(k:k+80),cmd(k:k+80),'-r')
%    
%    subplot(2,1,2)
%    plot(t(k:k+80),RH(k:k+80),'-o')
%    ylim([450,1600]);
%    xlim([t(k),t(k+95)]);
%    grid on;
%    
%    drawnow;
% %    pause(0.01);
% end
