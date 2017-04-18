%This script is used to perform simulations of the SMIB system

%System parameters
f = 60; %Frequency, Hz
w0 = 2*pi*f; %Synchronous speed, rad/s
H = 7 ; %Inertia constant, s*MW/MVA
D = -10; %Damping coefficient, p.u.


%System initialization

w(1) = 0; %initial value of rotor speed, p.u.
Pse(1) = Ep*V_IB/Xeq*sin(delta_0); %Initial value of synchronizing power
Pel(1) = Pse(1); %Initial value of electrical power
delta(1) = delta_0;


%Simulation parameters
dt = 0.001; %Time step
t_fin = 20; %Final simulation time
t_fault = 1; %Fault occurence time
t_pm_step = 1; % %Time instance for Pm step change - necessary for several experiments
dt_fault = 1; %Fault duration time
t_clear = t_fault + dt_fault; %Fault clearing time
time_elapsed = 0:dt:t_fin+dt; %Vector of elapsed time

i = 1; %Counter

%Numerical DE solver - main loop
for t = 0:dt:t_fin
     
%  if t > t_pm_step  %Pm step change
%      stability_limit=Ep*V_IB/Xeq;
%        Pm = stability_limit;
%  end

    %Modified Euler method
    %Preliminary step
    fd1 = w0*(w(i)-1); %Angle state RHS
    delta_next_step_p = delta(i)+fd1*dt;
    
%     %3-phase fault application
  if t >= t_fault && t < t_clear
     Pe = 0;
 else
     Pe = Ep*V_IB/Xeq*sin(delta(i))+D*(w(i) - 1);
 end

%  Pe = Ep*V_IB/Xeq*sin(delta(i))+D*(w(i) - 1); %Pe - #1
    fw1 = 1/2/H*(Pm - Pe);
    w_next_step_p = w(i) + fw1*dt;
    
    %Actual step
    fd2 = w0*(w_next_step_p-1);
    delta(i+1) = delta(i) + 0.5*dt*(fd1+fd2);
   
  %3-phase fault application
     if t >= t_fault && t < t_clear
       Pe = 0;
     else
        Pe = Ep*V_IB/Xeq*sin(delta_next_step_p)+D*(w_next_step_p - 1);
     end
%  Pe = Ep*V_IB/Xeq*sin(delta_next_step_p)+D*(w_next_step_p - 1); %Pe - #2
    fw2 = 1/2/H*(Pm - Pe); 
    w(i+1) = w(i)+0.5*dt*(fw1+fw2);
    Pse(i+1) = Ep*(V_IB/Xeq)*sin(delta(i+1)); %Synchronizing power
    Pel(i+1) = Ep*(V_IB/Xeq)*sin(delta(i+1)) + D*(w(i+1)-1); %Electrical power
    i = i+1;
end

% figure, plot(time_elapsed, delta), title('Angle vs Time'), xlabel('Time(s)'), ylabel('Angle')
% figure, plot(time_elapsed, w), title('Speed vs Time'), xlabel('Time(s)'), ylabel('Speed')
% figure, plot(time_elapsed, Pel), title('Electrical Power vs Time'), xlabel('Time(s)'), ylabel('Electrical Power')

% plot(time_elapsed, Pel), title('Electrical Power vs Time'), xlabel('Time(s)'), ylabel('Electrical Power(VA)')
% plot(time_elapsed, w), title('Speed vs Time'), xlabel('Time(s)'), ylabel('Speed(rad/s)')

% plot(time_elapsed, delta), title('Angle vs Time'), xlabel('Time(s)'), ylabel('Angle (Radians)')

%2.1 
% x=linspace(0,20, 100);
% y=zeros(1,100); 
% PmArray = ones(1,100)*Pm;
% plot(delta, Pel, '--b', x, y, '-o', x, PmArray, 'r'), title('Electrical Power vs Angle'), xlabel('delta'), ylabel('Electrical Power(pu)')
% legend('Electrical Power', 'Zero Power', 'Mechanical Power');
%  ylim([0,5])
%  xlim([0,20])
 
%Section 2.3, 2.4
%  plot(delta, w), xlabel('Rotor angle'), ylabel('Speed (w)'), title('Angle vs time when mechanical power is 90% of the steady state limit') 
%   plot(time_elapsed, delta), xlabel('Time(s)'), ylabel('Rotor Angle'), title('Angle vs time when mechanical power is 100% of the steady state limit') 
% plot(time_elapsed, w), xlabel('Time(s)'), ylabel('Speed'), title('Speed vs time when mechanical power is 100% of the steady state limit') 
%    plot(time_elapsed, Pel), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Power vs time when mechanical power is 100% of the steady state limit') 
% plot(delta, Pel, x, PmArray), xlabel('Rotor Angle)'), ylabel('Electrical Power'), title('Electrical Power vs rotor angle when mechanical power is 100% of the steady state limit') 
% plot( delta, time_elapsed), ylabel('Time(s)'), xlabel('Rotor Angle'), title('Time vs Rotor Angle when mechanical power is 100% of the steady state limit') 
% 

% ylim([0,5])
% hold on; 
% plot (x,y)
% 
% syms delta 
% resultDelta = solve(Pel ==0) 



% plot(log(time_elapsed), Pel)


%Section 3.2 

% plot(time_elapsed, delta), xlabel('Time(s)'), ylabel('Rotor Angle'), title('Angle vs time under 3-phase fault application.') 
% plot(delta, Pel, x, PmArray), xlabel('Rotor Angle'), ylabel('Electrical Power'), title('Electrical Power vs rotor angle under 3-phase fault application.') 
% ylim([-3,3])
% xlim([0,4])

%Section 4.2 
% plot(time_elapsed, delta), xlabel('Time(s)'), ylabel('Rotor Angle'), title('Angle vs time post disturbance') 
%  plot(log(time_elapsed), Pel), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Power vs time post disturbance') 
%  plot((time_elapsed), Pel), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Power vs time post disturbance') 
%  plot(time_elapsed, w), xlabel('Time(s)'), ylabel('Speed'), title('Speed vs time post disturbance') 

%Section 4.3
%Setting fault duration to 0.22 seconds. 
%  plot(log(time_elapsed), Pel), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Power vs time post disturbance') 
% plot((time_elapsed), Pel), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Power vs time under longer fault duration') 
% plot(time_elapsed, w), xlabel('Time(s)'), ylabel('Speed'), title('Speed vs time longer fault duration') 
% plot(time_elapsed, delta), xlabel('Time(s)'), ylabel('Rotor Angle'), title('Angle vs time longer fault duration') 

%Section 4.4 
% Pel_high = Pel; 
% Pel_low = Pel;
% Pel_high - Pel_low
% plot((time_elapsed), Pel_high, 'r', time_elapsed, Pel_low, 'b' ), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Powers comparison with different fault time duration')
% legend('fault duration of 0.22 seconds', 'fault duration of 0.05 seconds')

%Section 4.5
% plot((time_elapsed), Pel), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Power vs time with 20% of initial apparent power') 

%Section 4.6 
% clear x y 
% x=10;
% y=linspace(0,20, 100);
% plot((time_elapsed), Pel), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Power vs time with 20% of initial apparent power and D=24') 
% plot(time_elapsed, w), xlabel('Time(s)'), ylabel('Speed'), title('Speed vs time with 20% of initial apparent power and D=24') 
% plot(time_elapsed, delta), xlabel('Time(s)'), ylabel('Rotor Angle'), title('Angle vs time with 20% of initial apparent power and D=24') 

%Section 4.7%
% % plot((time_elapsed), Pel), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Power vs time with 20% of initial apparent power,  D=24, and fault duration of 1 second.') 
% 
% disp('Kinetic energy')
% Wk = H*w0*(w-1).^2; %Kinetic energy
% disp('Potential energy')
% Wp = -(Pm*(delta-delta_0)+Ep*V_IB/Xeq*(cos(delta)-cos(delta_0))); %Potential energy
% 
% W_tot= Wp +Wk;
% % plot(time_elapsed, Wp), xlabel('Time(s)'), ylabel('Potential energy Wp'), title('Potential energy vs time 20% of initial apparent power,  D=24, and fault duration of 1 second.') 
%  
% critical_energy = 53.41; 
% plot(time_elapsed, W_tot, time_elapsed , 53.41), xlabel('Time(s)'), ylabel('Total energy Wp+Wk'), title('Total energy vs time 20% of initial apparent power,  D=24, and fault duration of 1 second.') 
% ylim([51,54])
% xlim([2.8, 3.3])

%SQ 4.1.3 
plot((time_elapsed), Pel), xlabel('Time(s)'), ylabel('Electrical Power'), title('Electrical Power vs time with 20% of initial apparent power and D=24') 

