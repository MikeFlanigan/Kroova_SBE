%% SBE_data_viz_working

clear; clc; close all

% day two sailing data
% data = csvread("data/7-20/Moth_Data_2000-1-1_0h55m39s.csv"); % run 1
% data = csvread("data/7-20/Moth_Data_2000-1-1_0h57m23s.csv"); % run 2 
% data = csvread("data/7-20/Moth_Data_2000-1-1_0h58m54s.csv"); % run 3 
% data = csvread("data/7-20/Moth_Data_2000-1-1_1h1m19s.csv"); % run 4

% day three, successful foiling
% data = csvread("data/7-21/Moth_Data_2000-1-1_0h23m11s.csv"); 
% data = csvread("data/7-21/Moth_Data_2000-1-1_0h40m28s.csv"); 
data = csvread("data/7-21/Moth_Data_2000-1-1_0h44m3s.csv");
% data = csvread("data/7-21/Moth_Data_2000-1-1_0h48m11s.csv"); 

% data =      data(5:end,:); % truncate initial data

% micros =    data(:,1);
% poten =     data(:,2);
% US =        data(:,3);
% GPS_spd =   data(:,4);
% Heave_acc = data(:,5);
% Heel_ang =  data(:,6);
% Pitch_ang = data(:,7);

micros =    data(:,1);  % microseconds
poten =     data(:,2);  % wand movement 
ctl_in_sig = data(:,3); % control input 
flap_perc = data(:,4);  % flap output cmd percent 
US =        data(:,5);  % US signal 
GPS_spd =   data(:,6);  % GPS spd 
Heave_acc = data(:,7);  % Heave acceleration 
Heel_ang =  data(:,8);  % heel angle 
Pitch_ang = data(:,9);  % pitch angle 

%% get a time array
t(1) = 0;
for k = 1:length(data(:,1))-1
    if micros(k+1) > micros(k)
        delta_t(k) = (micros(k+1)-micros(k))/1000;
    else
        delta_t(k) = (micros(k+1) + 1000000 - micros(k))/1000;
    end
    delta_t(k) = delta_t(k)/1000; % seconds rather than ms
    t(k+1) = t(k)+delta_t(k);
end

disp(['mean ctrl loop time: ',num2str(mean(delta_t))])
disp(['maximum ctrl loop time: ',num2str(max(delta_t))])
disp(['minimum ctrl loop time: ',num2str(min(delta_t))])
disp(['stand dev ctrl loop time: ',num2str(std(delta_t))])

%% plot baseline overview useful plots
figure()
subplot(4,1,1)
plot(t,GPS_spd)
grid on
xlabel('t [seconds]')
ylabel('gps speed [knots]')

subplot(4,1,2)
plot(t,(1-poten));
grid on
xlabel('t [seconds]')
ylabel('Wand position')

subplot(4,1,3)
plot(t,US,'b-o')
grid on
ylim([0, 2000])
xlabel('t [seconds]')
ylabel('U.S. ride height [mm]')

subplot(4,1,4)
plot(t,Heel_ang)
grid on 
hold on 
plot(t,Pitch_ang)
xlabel('t [seconds]')
ylabel('Angle [degrees]')
legend('heel','pitch')

%% US filtering

US_filt(1) = US(1);
mem_weight = 0.05;
win_size = 50;
US_rav(1:win_size) = US(1:win_size);
for k = 1:length(data(:,1))-1
    if US(k+1) < 10
        US(k+1) = US(k);
    end
    US_filt(k+1) = US_filt(k)*(1-mem_weight)+US(k+1)*mem_weight;
    if k >= win_size
        US_rav(k+1) = mean(US(k-win_size+1:k));
    end
end

figure()
plot(t,US,'bo')
grid on
hold on
plot(t,US_filt,'g')
plot(t,US_rav,'r')
legend('US raw','US comp filt','US roll avg')
ylim([0, 2000])
xlabel('t [seconds]')
ylabel('U.S. ride height [mm]')
%% KF -- only update when new US data is in

% discrete time state transition matrix
F = zeros(3); % will update shortly
n = size(F,1);
% H measurement matrix
H = [1,0,0;
     0,0,1];
% process noise definition
% Omega = dt*[0,0,0;1,0,0;0,0,0;0,1,0;0,0,0;0,0,1];
% Q = diag([5,5,5]);
% Qkf = Omega*Q*Omega';
Qkf = [0.1, 0, 0;
       0, 0.01, 0;
       0, 0, 0.01];
% measurement noise definition 
R = [.100,0;
     0,0.01]; 
 P(:,:,1) = 1*diag([0.200,0.2,0.02]);
% x = RH, RHdot, RHddot
x(:,1) = [US(1)/1000; 0.0; Heave_acc(1)];
for k = 1:length(data(:,1))-1 
        % recompute F since it is time delta dependent 
        F = [1, delta_t(k), 0.5*delta_t(k)^2;
            0, 1, delta_t(k);
            0, 0, 1];

        %%Perform prediction step
        x_minus = F*x(:,k);
        P_minus = F*P(:,:,k)*transpose(F) + Qkf;

        %%Compute Kalman gain
        K = P_minus*transpose(H)/(H*P_minus*transpose(H)+ R);

        % gather measurement vector
        y(:,k) = [US(k)/1000; Heave_acc(k)];

        %%Perform measurement update step
        x(:,k+1) = x_minus + K*(y(:,k) - H*x_minus);
        P(:,:,k+1) = (eye(n) - K*H)*P_minus;
end

% on figure 2
plot(t,x(1,:)*1000,'m--')
legend('US raw','US comp filt','US roll avg','KF est')

figure()
plot(t,x(1,:),'m--')
hold on
grid on
plot(t,x(2,:),'r--')
plot(t,x(3,:),'g--')
plot(t,US/1000,'k--')
legend('pos','vel','acc','US RH')

%% KF -- update velocity when U.S. data is crappy

% % discrete time state transition matrix
% F = zeros(3); % will update shortly
% n = size(F,1);
% % H measurement matrix
% H = [1,0,0;
%      0,0,1];
% % process noise definition
% % Omega = dt*[0,0,0;1,0,0;0,0,0;0,1,0;0,0,0;0,0,1];
% % Q = diag([5,5,5]);
% % Qkf = Omega*Q*Omega';
% Qkf = [0.1, 0, 0;
%        0, 0.01, 0;
%        0, 0, 0.01];
% % measurement noise definition 
% R = [.010,0;
%      0,0.001]; 
%  P(:,:,1) = 1*diag([0.200,0.2,0.02]);
% % x = RH, RHdot, RHddot
% x(:,1) = [US(1)/1000; 0.0; Heave_acc(1)];
% 
% USflag = 0;
% klast = 1;
% ii = 1;
% tlog(1) = t(1);
% dt = 0;
% for k = 1:length(data(:,1))-1 
%     if USflag ~= US(k)/1000
%         dt = t(k)-t(klast);
%         logdt(ii) = dt;
%         % recompute F since it is time delta dependent 
%         F = [1, dt, 0.5*dt^2;
%             0, 1, dt;
%             0, 0, 1];
% 
%         %%Perform prediction step
%         x_minus = F*x(:,ii);
%         P_minus = F*P(:,:,ii)*transpose(F) + Qkf;
% 
%         %%Compute Kalman gain
%         K = P_minus*transpose(H)/(H*P_minus*transpose(H)+ R);
% 
%         % gather measurement vector
%         y(:,ii) = [US(k)/1000; Heave_acc(k)];
% 
%         %%Perform measurement update step
%         x(:,ii+1) = x_minus + K*(y(:,ii) - H*x_minus);
%         P(:,:,ii+1) = (eye(n) - K*H)*P_minus;
%         
%         tlog(ii) = t(k);
%         klast = k;
%         ii = ii+1;
%     else
%         % could continue to integrate or average or update the imu data
%         % here....
%         x(3,ii) = Heave_acc(k);
%         x(2,ii) = x(2,ii) + Heave_acc(k)*delta_t(k);
%         x(1,ii) =  x(1,ii) + x(2,ii)*delta_t(k) + 0.5*Heave_acc(k)*delta_t(k)^2;
%     end
% 
%     USflag = US(k)/1000;
% end
% 
% % on figure 2
% plot(tlog,x(1,2:end)*1000,'m--')
% legend('US raw','US comp filt','US roll avg','KF est')
% 
% figure()
% plot(tlog,x(1,2:end),'m--')
% hold on
% grid on
% plot(tlog,x(2,2:end),'r--')
% plot(tlog,x(3,2:end),'g--')
% plot(t,US/1000,'k--')
% legend('pos','vel','acc','US RH')
% 
% figure()
% histogram(logdt)