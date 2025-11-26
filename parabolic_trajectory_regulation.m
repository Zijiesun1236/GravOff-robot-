clear, clc, close all
rad=pi./180;
deg=180./pi;

%% initialization of LUSTER motion capture system
ip='192.xxx.x.xxx'; % define the ip of your own
luster = LuMoSDKMPI(ip);
luster.connect();
%% wireless serial setup (HC12)
s = serialport('COM7',115200);
s.DataBits = 8;
s.StopBits = 1;  
s.Parity = 'none';

%% initial conditions
v0=2.1; %  theoretical, unit: m/s
m= 0.367; % mass
inclination=20.*rad; % absolute value of inclination angle
alpha=pi./2-inclination;  %theoretical takeoff angle
psi0=0;
theta0=0; 
phi0=0; 
epsi0=inclination;

% equivalent gravity
g_equi=0.278*9.81; g=9.81;
Uz0=(m*g-m*g_equi); %intial thrust force 2.599 N
% assign initial value for thetat phit epsit
thetat=theta0;
phit=phi0;


%% discrete error state-space model
% error states: x1,x2 lateral,forward position, x3 x4: lateral,forward velocity
%horizontal
Ts=0.02; 
Axy=[1 0 Ts 0;0 1 0 Ts;0 0 1 0;0 0 0 1]; 
Bxy=[0 0; 0 0; Ts.*U1t./m 0 ; 0 Ts.*U1t./m]; 
Cxy=eye(4);
Dxy=0;

%vertical
cd=-0.0625; % vertical-speed-related damping factor
Az=[1, Ts; 0, 1+cd*Ts./m];
Bz=[0; Ts*(sin(epsit)*cos(thetat)*sin(phit) + cos(epsit)*cos(thetat)*cos(phit))./m];
Bud=[0; Ts*(sin(epsit)*cos(thetat)*sin(phit) + cos(epsit)*cos(thetat)*cos(phit))./m]; % thrust noise as unmeasured disturbance
Bmd=[0; Ts];
Cz=eye(4);
Dz=[0 0 0;0 0 0];

%%
%define the discrete ss models and MPC objects
model_xy=ss(Axy,Bxy,Cxy,Dxy,Ts);
model_z=ss(Az,[Bz,Bud,Bmd],Cz,Dz,Ts);
Ph=10;%prediction horizon
Ch=2;%control horizon

W_MV_z=1;  MVRate_z=0.08;   W_ov_z=[2.2,0]; % Qz: W_ov_z, Rz: W_MV_z, Rz,¦¤U: MVRate_z   
Weights_z = struct('MV',W_MV_z,'MVRate',MVRate_z,'OV',W_ov_z); % [300 8]| 0.5 0.08[10 1]
MV_z = struct('Min',{-1.0},'Max',{2.0},'ScaleFactor',{1});
OV_z = struct('Min',{-Inf,-Inf},'Max',{Inf,Inf},'ScaleFactor',{1,1}); %unconstrained outputs 

W_MV_xy=[1,1];   MVRate_xy=[0.3 0.3];    W_ov_xy=[2.9 2.5 0.2 0.5]; %trajec_track:[1 1][0.3 0.3][3.5 2 0.2 0.2]  wind_disturb_test=[1 0.8]; [0.3 0.3]; [3.5 6 0.2 0.35]
Weights_xy = struct('MV',W_MV_xy,'MVRate',MVRate_xy,'OV',W_ov_xy); %[15 15 2 2]; [8 15 0.25 0.25]
MV_xy = struct('Min',{-sin(10.*rad),-sin(30.*rad)},'Max',{sin(10.*rad),sin(30.*rad)},'ScaleFactor',{1,1}); % ux uy
OV_xy = struct('Min',{-Inf,-Inf,-Inf,-Inf},'Max',{Inf,Inf,Inf,Inf},'ScaleFactor',{1,1,1,1});
% 
model_z=setmpcsignals(model_z,'MV',1,'UD',2,'MD',3); % channel 1: manipulated variable ¦²T, channel 2: noise, channel 3: measured disturbance

MPCobj_z=mpc(model_z,Ts,Ph,Ch,Weights_z,MV_z,OV_z,[]);
MPCobj_xy=mpc(model_xy,Ts,Ph,Ch,Weights_xy,MV_xy,OV_xy,[]);

%% heading and position initialization 

frameData = luster.receiveFrame();
yaw0=frameData.allRigidbody(1).fYEulerAngle.*rad; 
psi_des=frameData.allRigidbody(1).fYEulerAngle;
if (psi_des<=0)
    psi_des=360+frameData.allRigidbody(1).fYEulerAngle;
end
R=[cos(yaw0),cos(pi./2-yaw0),0;cos(pi./2+yaw0),cos(yaw0),0;0,0,1]; %rotation matrix from global to local coordinates
xinG=0.001*frameData.allRigidbody(1).X;
yinG=0.001*frameData.allRigidbody(1).Y;
zinG=0.001*frameData.allRigidbody(1).Z;
G_L=[xinG;yinG;0]; % displacement matrix from G to L 
X_L=[];Y_L=[];Z_L=[];VX_L=[]; VY_L=[]; VZ_L=[]; T=[];X_R=[];Y_R=[];Z_R=[];X_ALL=[];Y_ALL=[];Z_ALL=[];T_ALL=[];Acc_z=[];
Z_ERR_LOG=[];
XY_ERR_LOG=[];
info_z=[];
info_xy=[];
Thrust_log=[];
Theta_log=[];Theta_real_log=[];
Phi_log=[];Phi_real_log=[];PHI_ALL=[]; THETA_ALL=[]; PSI_ALL=[];Epsi_log=[];
X_pred=[]; Y_pred=[]; Z_pred=[];
err=[];
t_log=[];
z_compare_baffer=[];h_max=[];

%% define mpcstate and nominals, as arguments in mpcsolver 

Xz=mpcstate(MPCobj_z);
Xxy=mpcstate(MPCobj_xy);
nominal_z=MPCobj_z.Model.Nominal;
nominals_z=repmat(nominal_z,Ph+1,1);
nominal_xy=MPCobj_xy.Model.Nominal;
nominals_xy=repmat(nominal_xy,Ph+1,1);

%%  set unmeasured input disturbance (random walk noise)

Az_n = 1;   % x_id(k+1) = A x_id(k) + B w_id(k)  w_id(k) white noise with zero mean and unit variance.
Bz_n = 0.05;
Cz_n = 1;   % d(k) = Cid x_id(k) + Did w_id(k) 
Dz_n = [];

% Define and assign disturbance model(random walk noise defined by Az_id, Bz_id, Cz_id, Dz_id)
disturbance_model_z = ss(Az_n, Bz_n, Cz_n , Dz_n, Ts);
setindist(MPCobj_z, 'model', disturbance_model_z);

% initial value definition
z0=0.20; 
y0=0.20*tan(alpha);

%%
land_obj_h=0; % the targetted object height to be landed on 
t_all_st=tic; % start counting time 
while (true)
    frameData = luster.receiveFrame(); 
    zinG_pre=0.001*frameData.allRigidbody(1).Z;
    xinG_pre=0.001*frameData.allRigidbody(1).X;
    yinG_pre=0.001*frameData.allRigidbody(1).Y;
    azinG_pre=frameData.allRigidbody(1).fZAcceleration;
    theta_all=-frameData.allRigidbody(1).fZEulerAngle; phi_all=frameData.allRigidbody(1).fXEulerAngle; psi_all=frameData.allRigidbody(1).fYEulerAngle;% ¼ÓÁË¸ººÅ
    THETA_ALL=[THETA_ALL,theta_all]; PHI_ALL=[PHI_ALL,phi_all]; PSI_ALL=[PSI_ALL,psi_all];
    pinL_pre=R*([xinG_pre;yinG_pre;zinG_pre]-G_L);
    double(zinG_pre);     
    X_ALL=[X_ALL,pinL_pre(1)]; Y_ALL=[Y_ALL,pinL_pre(2)];Z_ALL=[Z_ALL,pinL_pre(3)];Acc_z=[Acc_z,azinG_pre];
        t_all=toc(t_all_st) 
        T_ALL=[T_ALL,t_all];
    
    if zinG_pre>=z0 %land_obj_h+z0         
        break
    end 
    U1t=Uz0; phit=0; thetat=0; psi_err=0; epsit=epsi0; damp_st_indi=0;
    str1=num2str(U1t,'%0.2f');
    str2=num2str(phit,'%0.2f');
    str3=num2str(thetat*deg,'%0.2f');
    str4=num2str(psi_err,'%0.2f'); 
    str5=num2str(damp_st_indi);% str5: damping ready indi,0 off,1 on
    str6=num2str(epsit*deg,'%0.2f');
    w_tar=(v0-0.0736497)./0.00256049; 
    str7=num2str(w_tar,'%0.2f');
    write(s,[str1,'!',str2,'$',str3,'~',str4,'#',str5,'@',str6,'^',str7,'`'],'char'); % in this while loop, these values are transferred and assigned but not applied, in this way the leg extension speed can be known by low-level controller prior to jump.
end 

t_mpc=tic; % start counting time (for the mpc loop)
t=0;t_prev=0;Delta_T=[];
est_e=2;mea_e=4;q=0.0002;prev_est_Vx=0;prev_est_Vy=0;prev_est_Vz=0;climb_finished=0;
prev_h=zinG_pre;curr_h=zinG_pre;descend_indi=0;
epsit_prev=0;
VINLT=[];EST_V=[];
SWITCH_INDI=[];
DAMP=[];damp_st_indi=0;
Switch_time=NaN;
azinGt=azinG_pre;
switch_indi=0;
switch_indi_lock=0;
YAW_LOG=[];
for k=1:1:round(2*(v0*sin(alpha)./g_equi)./0.013) % time elapsed in a parabolic trajectory, divided by 0.013 (not 0.02) for some time margin; loop will end till robot exit the "h_thres=0.2 m" line.
t_prev=t;
t_log=[t_log,t];
prev_h=curr_h;
%% getting the current state
frameData = luster.receiveFrame();
psit=frameData.allRigidbody(1).fYEulerAngle;
    if(psit<0)
    psit=360+psit;
    end
   
xinGt=0.001*frameData.allRigidbody(1).X; vxinGt=frameData.allRigidbody(1).fXSpeed;
yinGt=0.001*frameData.allRigidbody(1).Y; vyinGt=frameData.allRigidbody(1).fYSpeed;
zinGt=0.001*frameData.allRigidbody(1).Z; vzinGt=frameData.allRigidbody(1).fZSpeed; 
az_prev=azinGt; 
azinGt=frameData.allRigidbody(1).fZAcceleration;
d_az=azinGt-az_prev;
pinLt=R*([xinGt;yinGt;zinGt]-G_L); 
vinLt=R*[vxinGt;vyinGt;vzinGt]; % **check direction 
curr_h=zinGt;
dh=curr_h-prev_h;
   %% unidirectional kalman filter
    est_e1=est_e;est_e2=est_e;est_e3=est_e;
    %define kalman gain
    Mx=est_e1./(est_e1+mea_e); My=est_e2./(est_e2+mea_e);Mz=est_e3./(est_e3+mea_e);
    if (k==1)
        curr_est_Vx=vinLt(1);curr_est_Vy=vinLt(2);curr_est_Vz=vinLt(3);
    else
        curr_est_Vx=prev_est_Vx + Mx*(vinLt(1)-prev_est_Vx);
        curr_est_Vy=prev_est_Vy + My*(vinLt(2)-prev_est_Vy);
        curr_est_Vz=prev_est_Vz + Mz*(vinLt(3)-prev_est_Vz);
    end
    %update the estimate err
    est_e1=(1.0-Mx)*est_e1+abs(prev_est_Vx-curr_est_Vx)*q;
    est_e2=(1.0-My)*est_e2+abs(prev_est_Vy-curr_est_Vy)*q;
    est_e3=(1.0-Mz)*est_e3+abs(prev_est_Vz-curr_est_Vz)*q;
    prev_est_Vx=curr_est_Vx;prev_est_Vy=curr_est_Vy;prev_est_Vz=curr_est_Vz;

    %%

theta_real=-rad*frameData.allRigidbody(1).fZEulerAngle; % add "-" to reverse the reads from mocap(due to coordinate orientation difference in mocap), remove "-" if your mocap system has the same orientation as your defined one.
phi_real=rad*frameData.allRigidbody(1).fXEulerAngle; 
psi_err=psi_des-psit;
psi_err=round(psi_err,2);
% YAW_LOG=[YAW_LOG, psi_err]
t=toc(t_mpc)
dt=t-t_prev; Delta_T=[Delta_T,dt];
t_all=toc(t_all_st);
T_ALL=[T_ALL,t_all];

%% z ,xy direction reference trajectory
% z
Xzr=v0.*sin(alpha).*t-g_equi./2.*t.^2 + z0;
Vzr=v0.*sin(alpha)-t.*g_equi;
% x
Xxr=0;
Vxr=0;
% y
Xyr=v0.*cos(alpha).*t+y0;
Vyr=v0.*cos(alpha);

% current measured output in z and xy direction
S_CURR_Z=double([pinLt(3)-Xzr;curr_est_Vz-Vzr]); %z~,z~dot
S_CURR_XY=double([pinLt(1)-Xxr;pinLt(2)-Xyr;curr_est_Vx-Vxr;curr_est_Vy-Vyr]); % x~, y~, x~dot, y~dot 

%% update Bz(k) and compute Uz
% ***assign the initial value for the state estimator
epsit_prev=epsit;
Xz.Plant=S_CURR_Z; % assign the current actual robot states for the mpcstate instance Xz

% speed-related disturbance force vector, as measured disturbance
vz=@(k)(v0.*sin(Alpha)-k.*Ts.*g_equi); % vz: vertical speed at k-th step
MD=[c*vz(k)./m; c*vz(k+1)./m;c*vz(k+2)./m;c*vz(k+3)./m;c*vz(k+4)./m;c*vz(k+5)./m;c*vz(k+6)./m;c*vz(k+7)./m;c*vz(k+8)./m;c*vz(k+9)./m;c*vz(k+10)./m];

% fixed rotor
% Bz_real= [0 0 0; Ts*cos(theta_real).*cos(phi_real)./m  Ts*cos(theta_real).*cos(phi_real)./m  Ts];% consider the case with disturbance model

% vectoring mode
% Bz_real: Bz realtime
Bz_real= [0 0 0; Ts*(sin(epsit_prev)*cos(theta_real)*sin(phi_real) + cos(epsit_prev)*cos(theta_real)*cos(phi_real))./m  Ts*(sin(epsit_prev)*cos(theta_real)*sin(phi_real) + cos(epsit_prev)*cos(theta_real)*cos(phi_real))./m  Ts];  % vector mode 
model_z=ss(Az,Bz_real,Cz,Dz,Ts);  
[MV_curr_z,info_z]=mpcmoveAdaptive(MPCobj_z, Xz, model_z, nominals_z,S_CURR_Z,[0;0],MD); % mpc solver, output ¦²T~ and info_z (including the state predictions) 

%% update Bxy(k) and compute Ux and Uy
% assign the current actual robot states for the mpcstate instance Xz
Xxy.Plant=S_CURR_XY;
U1t=Uz0+MV_curr_z; % actual thrust= ref_thrust + ¦²T~
Bxy_real=[0 0; 0 0 ;  Ts*U1t./m 0; 0 Ts*U1t./m];
model_xy=ss(Axy,Bxy_real,Cxy,Dxy,Ts);
[MV_curr_xy,info_xy]=mpcmoveAdaptive(MPCobj_xy,Xxy,model_xy,nominals_xy,S_CURR_XY,[0;0;0;0],[]); 
ux=MV_curr_xy(1,:);uy=MV_curr_xy(2,:);% x-y direction mpc outputs.

%% command calculation for fixed-rotor mode
% stheta_cphi=(ux+uy.*sin(psi_err*hd)./cos(psi_err*hd))./(cos(psi_err*hd)+sin(psi_err*hd).^2./cos(psi_err*hd));
% phit=asin((sin(psi_err*hd).*stheta_cphi-uy)./cos(psi_err*hd));
% thetat=asin(stheta_cphi./cos(phit));
% phit=-phit; thetat=-thetat;% reverse direction (due to mocap local coordinate definition difference)
% phit=phit*deg; thetat=thetat*deg;
%% command calculation for thrust vectoring mode
A=sin(psi_err*rad).^2.*cos(phi_real)./cos(psi_err*rad) + cos(phi_real).*cos(psi_err*rad);
B=sin(psi_err*rad).^2.*sin(phi_real)./cos(psi_err*rad) + sin(phi_real).*cos(psi_err*rad);
C=sin(psi_err*rad)./cos(psi_err*rad).*ux-uy;
varphi=atan(B/A);
epsit= asin(C./sqrt(A.^2 + B.^2))-varphi;
thetat=asin((ux-sin(epsit)*sin(psi_err*rad)*cos(phi_real) - cos(epsit)*sin(phi_real)*sin(psi_err*rad))/ (cos(epsit)*cos(phi_real)*cos(psi_err*rad)-sin(phi_real)*cos(psi_err*rad)*sin(epsit)));

%% assigning usr-defined phit: here a step change occurs near apex  
 if dh>0.013 && switch_indi_lock==0
     switch_indi=0;
     phit=-20;
 elseif dh<0.013 && t>0.3
     switch_indi=1;
     switch_indi_lock=1;
 end
 if switch_indi==1
 phit=20;
 end
 SWITCH_INDI=[SWITCH_INDI,switch_indi];
 if SWITCH_INDI(k)==1 && SWITCH_INDI(k-1)==0
     Switch_time=t;
 end
 
 
 %% find peak height h_max
% find apex h_max
z_compare_baffer=[z_compare_baffer, zinGt];
if numel(z_compare_baffer)==7
    if max(z_compare_baffer)==z_compare_baffer(:,4) 
        h_max=z_compare_baffer(:,4);
    end
end
if numel(z_compare_baffer)>= 7
    z_compare_baffer(:,1)=[];
end

 %% active damping
 h_thres=land_obj_h+0.35; % 0.35 > stand height 0.3, leave 0.05s for signal transfer
 if ~isempty(h_max)
     v_land=sqrt(2*g_equi*(h_max-h_thres)+v0^2*sin(abs(inclination))^2);
 end
 % set damp_st_indi
 if dh<-0.001 && zinGt>h_thres  % desending but higher than h_thres
    damp_st_indi=2; % leg motor soft activation
 end
 if dh<-0.001 && zinGt<=h_thres % descending and lower than h_thres
    damp_st_indi=1;
    w_tar=0.85*(v_land-0.0736497)/0.00256049;% derived from statistical recorded linear relationship between v0 and w_leg, see Fig. S7
 end
 DAMP=[DAMP, damp_st_indi];
%% assign outputs as string and send to wireless serial HC12
str1=num2str(U1t,'%0.2f');
str2=num2str(phit,'%0.2f');
str3=num2str(thetat*deg,'%0.2f');
str4=num2str(psi_err,'%0.2f'); 
str5=num2str(damp_st_indi);% str5: damping state indicator,0 off,1 on, 2 soft activation
str6=num2str(epsit*deg,'%0.2f'); 
str7=num2str(w_tar,'%0.2f');
write(s,[str1,'!',str2,'$',str3,'~',str4,'#',str5,'@',str6,'^',str7,'`'],'char');

%% logging
% get predictions 
x_pred=info_xy.Yopt(:,1);y_pred=info_xy.Yopt(:,2); z_pred=info_z.Yopt(:,1); vz_pred=info_z.Yopt(:,2);
vx_pred=info_xy.Yopt(:,3);vy_pred=info_xy.Yopt(:,4);
X_pred=[X_pred,x_pred]; Y_pred=[Y_pred,y_pred];Z_pred=[Z_pred,z_pred];

Thrust_log=[Thrust_log,U1t];
Theta_log=[Theta_log,thetat*deg];
Theta_real_log=[Theta_real_log,theta_real];
Phi_log=[Phi_log,phit];
Phi_real_log=[Phi_real_log,phi_real];
Epsi_log=[Epsi_log, epsit*deg];
Z_ERR_LOG=[Z_ERR_LOG, S_CURR_Z];
XY_ERR_LOG=[XY_ERR_LOG, S_CURR_XY];
X_L=[X_L,pinLt(1)];X_ALL=[X_ALL,pinLt(1)]; X_R=[X_R,Xxr];
Y_L=[Y_L,pinLt(2)];Y_ALL=[Y_ALL,pinLt(2)]; Y_R=[Y_R,Xyr];
Z_L=[Z_L,pinLt(3)];Z_ALL=[Z_ALL,pinLt(3)]; Z_R=[Z_R,Xzr];
Acc_z=[Acc_z,azinGt];
THETA_ALL=[THETA_ALL, theta_real]; PHI_ALL=[PHI_ALL,phi_real]; PSI_ALL=[PSI_ALL, psit];

%% exit condition   
if zinGt<0.2 && t>0.4 % exit condition of the mpc loop
   break;
end

end

level_indi=0;
% this "for loop" records the later states (e.g., x y z positions) and assign open-loop commands 
for land_count=1:60 
frameData = luster.receiveFrame();
    zinGt=0.001*frameData.allRigidbody(1).Z;   vzinGt=frameData.allRigidbody(1).fZSpeed;
    xinGt=0.001*frameData.allRigidbody(1).X;   vxinGt=frameData.allRigidbody(1).fXSpeed;
    yinGt=0.001*frameData.allRigidbody(1).Y;   vyinGt=frameData.allRigidbody(1).fYSpeed;
    theta_real=-frameData.allRigidbody(1).fZEulerAngle; phi_real=frameData.allRigidbody(1).fXEulerAngle; psit=frameData.allRigidbody(1).fYEulerAngle;
    azinGt=frameData.allRigidbody(1).fZAcceleration;
    pinLt=R*([xinGt;yinGt;zinGt]-G_L);
    vinLt=R*([vxinGt; vyinGt; vzinGt]);
    if(psit<0) % ensure the yaw angle ranges from 0-360 (in mocap it uses -180 to 180)
    psit=360+psit; 
    end    
    psi_err=psi_des-psit;
    t_all=toc(t_all_st); 
    U1t=Uz0;
    thetat=0;
    psi_err=round(psi_err,2);
    damp_st_indi=1;
    if zinGt<=0.4 && zinGt>0.12 && level_indi==0 % hold previous posture
        phit=20;
        epsit=rad*(-20);
    elseif zinGt<=0.12 % maintain horizontal near contact point
        phit=0;
        epsit=rad*(0);
        level_indi=1;
    end
    str1=num2str(U1t,'%0.2f');
    str2=num2str(phit,'%0.2f');
    str3=num2str(thetat*deg,'%0.2f');
    str4=num2str(psi_err,'%0.2f'); 
    str5=num2str(damp_st_indi);
    str6=num2str(epsit*deg,'%0.2f'); 
    str7=num2str(w_tar); % not assigned to the robot, as jumping terminated in teensy
    
    write(s,[str1,'!',str2,'$',str3,'~',str4,'#',str5,'@',str6,'^',str7,'`'],'char');
    % data logging
    T_ALL=[T_ALL,t_all]; Acc_z=[Acc_z, azinGt];
    X_ALL=[X_ALL,pinLt(1)]; Y_ALL=[Y_ALL,pinLt(2)]; Z_ALL=[Z_ALL,pinLt(3)];
    THETA_ALL=[THETA_ALL,theta_real]; PHI_ALL=[PHI_ALL,phi_real]; PSI_ALL=[PSI_ALL,psi_err];
   
end
 
%% calculate the mean absolute error of predictions states (x,y,z) and the experimentally recorded trajectory
x_actual=XY_ERR_LOG(1,:);y_actual=XY_ERR_LOG(2,:);z_actual=Z_ERR_LOG(1,:);

MAE_X=[]; MAE_Y=[];MAE_Z=[];  
for i=1:numel(x_actual)
    if i<numel(x_actual)-Ph % <10 
    mae_x=sum(abs(X_pred(:,i)'-x_actual(i:i+Ph)))./(Ph);
    mae_y=sum(abs(Y_pred(:,i)'-y_actual(i:i+Ph)))./(Ph);
    mae_z=sum(abs(Z_pred(:,i)'-z_actual(i:i+Ph)))./(Ph);
    elseif i>=numel(x_actual)-Ph
    X_pred_rest=X_pred(:,i)'; mae_x=sum(abs(X_pred_rest(1:numel(x_actual)-i+1)-x_actual(i:end)))./(numel(x_actual)-i+1);      
    Y_pred_rest=Y_pred(:,i)'; mae_y=sum(abs(Y_pred_rest(1:numel(y_actual)-i+1)-y_actual(i:end)))./(numel(y_actual)-i+1);
    Z_pred_rest=Z_pred(:,i)'; mae_z=sum(abs(Z_pred_rest(1:numel(z_actual)-i+1)-z_actual(i:end)))./(numel(z_actual)-i+1);
    end
    
    MAE_X=[MAE_X,mae_x];
    MAE_Y=[MAE_Y,mae_y];
    MAE_Z=[MAE_Z,mae_z];
end
%%
figure(1)
scatter3(X_L,Y_L,Z_L,45); % recorded trajectory
hold on
scatter3(X_R,Y_R,Z_R,30); % reference trajectory
% model predictions
for i =1:numel(X_R)-10
plot3(X_pred(:,i)+X_R(i:i+10)',Y_pred(:,i)+Y_R(i:i+10)',Z_pred(:,i)+Z_R(i:i+10)','k');
hold on
end
zlim([0 1]);ylim([0 0.85]);
legend('real','reference','predictions')
grid on
xlabel('x(m)','FontName','Calibri','FontSize',15);
ylabel('y(m)','FontName','Calibri','FontSize',15);
zlabel('z(m)','FontName','Calibri','FontSize',15);
set(gca,'FontSize',15);

figure(2)
subplot(4, 2, 1);
plot(t_log, Thrust_log,'b');
title('Overall thrust(N)');

subplot(4, 2, 2);
% Plot data in the second subplot
plot(t_log, Z_ERR_LOG(1,:)); 
title('Height err(m)');

subplot(4, 2, 3);
plot(t_log, Z_ERR_LOG(2,:),'black'); % z-direction velocity error
hold on
plot(t_log,XY_ERR_LOG(3,:),'green'); % x-direction velocity error
hold on 
plot(t_log,XY_ERR_LOG(4,:),'blue'); % y-direction velocity error
legend('z','x','y')
title('Velocity err(m/s)');


subplot(4,2,4);
plot(t_log,XY_ERR_LOG(1,:),'y'); % x-direction position error
hold on
plot(t_log,XY_ERR_LOG(2,:),'b'); % y-direction position error
title('Lateral and horizontal err(m)')
legend('lateral err','horizontal err')

subplot(4,2,5);
plot(t_log,Theta_log,'-k'); %theta command-time curve
hold on 
plot(t_log,Theta_real_log*deg,'-b'); % theta_real-time curve
title('pitch angle')
legend('command','real')

subplot(4,2,6);
plot(t_log,Phi_log,'-k');  % phi command-time curve
hold on 
plot(t_log,Phi_real_log*deg,'-b'); % phi_real-time curve
title('roll angle')
legend('command','real')

subplot(4,2,7);
plot(t_log,Epsi_log,'black'); % ¦År command
xlabel('Time/s'); ylabel('Epsilon/deg');

subplot(4,2,8);
plot(t_log,Z_L,'black'); % height curve (in mpc phase)
xlabel('Time/s'); ylabel('Z/m')

filtered_acc_z=smooth(T_ALL, Acc_z,0.05,'loess');

figure(3)
subplot(1,3,1)
scatter3(X_ALL,Y_ALL,Z_ALL,16); % complete path, including trajectory out of the MPC phase
axis equal
title("Complete trajectory")

subplot(1,3,2)
plot(T_ALL, filtered_acc_z);  % acc
xlabel('Time/s');
ylabel('Z Acceleration(m/s^2)');

subplot(1,3,3)
plot(T_ALL, Z_ALL); % height-time curve
ylabel('height/m');
xlabel('Time/s');
