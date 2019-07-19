%% SBE Simulations 

clear;
clc;

del_t = 0.1; % 10 Hz
time = 0; % run time

foil_period = 5; % seconds 
disp_period = 2; % seconds 

w_down = 50; % mm
w_up = 150; % mm

Foiling = 0;

foil_rh = 1200; % mm
disp_rh = 120; % mm
tr_RH = disp_rh;

while 1
    if Foiling == 1 && time >= foil_period
            time = 0;
            Foiling = 0;
    end
    if Foiling == 0 && time >= disp_period
            time = 0;
            Foiling = 1;
    end
    
    if Foiling == 1
        if tr_RH < foil_rh
            tr_RH = tr_RH + 130;
        end
    end
    if Foiling == 0
        if tr_RH > disp_rh
            tr_RH = tr_RH - 130;
        end
    end
    disp(tr_RH)
    
    time = time + del_t;
    Foiling
    pause(del_t)
end
