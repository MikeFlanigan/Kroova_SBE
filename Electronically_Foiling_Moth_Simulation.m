## Electronically_Foiling_Moth_Simulation
clear
clc

x = [1];
y = [3];

startx = 100;
starty = -100;
width = 1500;
height = 800;
plotting = true;
%plotting = false;
if plotting
  figure('Position',[startx,starty,width,height]);
endif
velocity = 5; # arbitrary unit / time 
t = .1; # pause time in seconds, should be related to velocity

init_ride_height = 2; 
sensor_position = 250;

water_surf(1:995) = 0;


function retlist = randmi
  retlist(1:2) = 0;
  retlist(1) = randi(5);
  retlist(2) = randi(5);
endfunction

function wave_particles = wave(init,period,amplitude)
  x = linspace(0,2*pi,100);
% x = 0:1/period:2*pi;
  wave_particles = sin(x)*amplitude+init;
endfunction

sc = 1; # speed count 
wave_particles = []; # initialization
function water_surf = update_water_surf(water_surf,wave_particles)
  if (length(wave_particles)==0)
    period = 5;
    amplitude = .5;
    wave_particles = wave(water_surf(end),period,amplitude);
%    display("called")
  endif
  water_surf = [water_surf(sc+1:end),wave_particles(1:sc)];
  wave_particles = [ave_particles(sc+1:end)];
%  display(max(water_surf))
endfunction

ride_height(1:500) = init_ride_height;
%function ride_height = sensor_read(water_surf,sc,init_ride_height);
%  ride_height = init_ride_height - water_surf(sensor_position:sensor_postion+sc);
%endfunction

elapsed = 0; # elapsed simulation time in seconds
while (elapsed < 1.5)
  x = randmi(1);
  y = randmi(2);

% ----------- could move to a function 
%  water_surf = update_water_surf(water_surf, wave_particles);
sc = 10; # speed count 
  if (length(wave_particles)==0 || length(wave_particles)<sc)
    period = 5;
    amplitude = .5;
%    wave_particles = wave(water_surf(end),period,amplitude);
    wave_particles = wave(0,period,amplitude);
%    display("called")
  endif
  water_surf = [water_surf(sc+1:end),wave_particles(1:sc)];
  wave_particles = [wave_particles(sc+1:end)];
%  display(max(water_surf))
% ----------- could move to a function 
  
  % meant to have a function call here
  ride_height = [ride_height(sc+1:end),init_ride_height - water_surf(sensor_position:sensor_postion+sc)];

  if plotting
    subplot(2,1,1);
    plot(water_surf,'bo')
    grid on 
    axis([0,1000,-5,10])
    subplot(2,1,2);
    plot(y,x,'ko')
    grid on
    axis([0,1000,-5,10])

    drawnow()
  endif
  pause(t)
  elapsed = elapsed + t;
endwhile