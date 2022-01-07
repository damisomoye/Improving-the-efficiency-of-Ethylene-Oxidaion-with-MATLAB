
%%
nlobj = nlmpc(4,1,'MV',1,'MD',2); % 4 states, 1 output, 1 manipulated variable 

%%
Ts = 5; %contoller sample time
PredictionHorizon = 10;
ControlHorizon = 3;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = PredictionHorizon; %specify prediction horizon
nlobj.ControlHorizon = ControlHorizon; %specify control horizon

%%
nlobj.Model.StateFcn = 'oxidationStateFcn';   %prediction model of the system
nlobj.States(1).Name  = 'Density';
nlobj.States(2).Name  = 'E_conc';
nlobj.States(3).Name  = 'EO_conc';
nlobj.States(4).Name  = 'T_reactor';

%%
% Specify the output function that returns the C2H4O concentration in the
% effluent flow (same as x3). Its scale factor is its typical operating
% range.  
nlobj.Model.OutputFcn = @(x,u) x(3);    
nlobj.OV.Name = 'CEOout';
nlobj.OV.ScaleFactor = 0.03;

%%
% Define manipulated variable using constraints from actuator limitation(operating range)
nlobj.MV.Min = 0.0704;
nlobj.MV.Max = 0.7042;
nlobj.MV.Name = 'Qin';
nlobj.MV.ScaleFactor = 0.6;

%%
% Define measured disturbance. 
nlobj.MD.Name = 'CEin';
nlobj.MD.ScaleFactor = 0.5;

%%
% The plant is Initially t at an equilibrium operating point with a low C2H4O (_y_ = |0.03|) conc 
% Using Optimization Toolbox, |fsolve| is used to find initial values of both the states and output
options = optimoptions('fsolve','Display','none');
uLow = [0.38 0.5];
xLow = fsolve(@(x) oxidationStateFcn(x,uLow),[1 0.3 0.03 1],options);
yLow = xLow(3);

%%
% Ensure prediction models is valid 
% Validate the functions at the initial state and output values using |validateFcns| command.
validateFcns(nlobj,xLow,uLow(1),uLow(2));

%%
% Specify the reference signal at the point it it increases from |0.03| to |0.05| at time |100|.
Tstop = 300;
time = (0:Ts:(Tstop+PredictionHorizon*Ts))';
r = [yLow*ones(sum(time<100),1);linspace(yLow,yLow+0.02,11)';(yLow+0.02)*ones(sum(time>150),1)];
ref.time = time;
ref.signals.values = r;

%%
% Use the Simulink model evaluate its performance. 
mdlNMPC = 'oxidationNMPC';
open_system(mdlNMPC)

%%
% Run and view the simulation
sim(mdlNMPC)
open_system([mdlNMPC '/y'])

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             Part B

%% Obtain Linear MPC Controllers from Nonlinear MPC Controller using convertToMPC

%%
%Use low C2H4O conversion rate |y| = |0.03| to generate linear MPC controller at an operating point
mpcobjLow = convertToMPC(nlobj,xLow,uLow); %specify operating point using state and input values

%%
% Use medium C2H4O conversion rate |y| = |0.04| to generate linear MPC controller .
uMedium = [0.24 0.5];
xMedium = fsolve(@(x) oxidationStateFcn(x,uMedium),[1 0.3 0.03 1],options);
mpcobjMedium = convertToMPC(nlobj,xMedium,uMedium);

%%
% Use high C2H4O conversion rate |y| = |0.05| to generate linear MPC controller
uHigh = [0.15 0.5];
xHigh = fsolve(@(x) oxidationStateFcn(x,uHigh),[1 0.3 0.03 1],options);
mpcobjHigh = convertToMPC(nlobj,xHigh,uHigh);

%% Feedback Control with Gain-Scheduled MPC
% Use MPC controllers above to implement a gain-scheduled MPC solution 

%Scheduling scheme conditions: 
% If y is lower than 0.035 -->  |mpcobjLow|.
% If y is higher than 0.045 --> |mpcobjHigh|.
% Otherwise, use |mpcobjMedium|.
%
% Evaluate the gain-scheduled controller performance sing simulink
mdlMPC = 'oxidationMPC';
open_system(mdlMPC)

%%
% Run and view simulation.
sim(mdlMPC)
open_system([mdlMPC '/y'])



