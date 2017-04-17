%This script is used to perform simulations of the SMIB system

%System parameters
f = 60; %Frequency, Hz
w0 = 2*pi*f; %Synchronous speed, rad/s
H = 4.5 ; %Inertia constant, s*MW/MVA
D = 0; %Damping coefficient, p.u.


%System initialization

w(1) = 0.5; %initial value of rotor speed, p.u.
Pse(1) = Ep*V_IB/Xeq*sin(delta_0); %Initial value of synchronizing power
Pel(1) = Pse(1); %Initial value of electrical power
delta(1) = delta_0;


%Simulation parameters
dt = 0.001; %Time step
t_fin = 10; %Final simulation time
t_fault = 1; %Fault occurence time
t_pm_step = 1; % %Time instance for Pm step change - necessary for several experiments
dt_fault = 0.1 ; %Fault duration time
t_clear = t_fault + dt_fault; %Fault clearing time
time_elapsed = 0:dt:t_fin+dt; %Vector of elapsed time

i = 1; %Counter

%Numerical DE solver - main loop
for t = 0:dt:t_fin
     
%     if t > t_pm_step  %Pm step change
%         Pm = ;
%     end

    %Modified Euler method
    %Preliminary step
    fd1 = w0*(w(i)-1); %Angle state RHS
    delta_next_step_p = delta(i)+fd1*dt;
    
%     %3-phase fault application
%     if t >= t_fault && t < t_clear
%         Pe = 0;
%     else
%         Pe = Ep*V_IB/Xeq*sin(delta(i))+D*(w(i) - 1);
%     end

    Pe = Ep*V_IB/Xeq*sin(delta(i))+D*(w(i) - 1); %Pe - #1
    fw1 = 1/2/H*(Pm - Pe);
    w_next_step_p = w(i) + fw1*dt;
    
    %Actual step
    fd2 = w0*(w_next_step_p-1);
    delta(i+1) = delta(i) + 0.5*dt*(fd1+fd2);
   
%     %3-phase fault application
%     if t >= t_fault && t < t_clear
%         Pe = 0;
%     else
%         Pe = Ep*V_IB/Xeq*sin(delta_next_step_p)+D*(w_next_step_p - 1);
%     end
    
    Pe = Ep*V_IB/Xeq*sin(delta_next_step_p)+D*(w_next_step_p - 1); %Pe - #2
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
x=linspace(0,4, 100)
y=zeros(1,100); 
plot(delta, Pel, x, y), title('Electrical Power vs Angle'), xlabel('Angle (rad)'), ylabel('Electrical Power(VA)')
 xlim([0,10])
 xlim([0,3.5])
% hold on; 
% plot (x,y)
% 
% syms delta 
% resultDelta = solve(Pel ==0) 



% plot(log(time_elapsed), Pel)


