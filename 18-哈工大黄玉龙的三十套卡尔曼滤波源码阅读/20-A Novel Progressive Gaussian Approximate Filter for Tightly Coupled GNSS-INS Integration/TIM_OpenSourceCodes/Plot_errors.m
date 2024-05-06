function Plot_errors(errors1,errors2)
%Plots navigation solution errors
%
% Input:
%   errors      Array of error data to plot
% Format is
% Column 1: time (sec)
% Column 2: north position error (m)
% Column 3: east position error (m)
% Column 4: down position error (m)
% Column 5: north velocity (m/s)
% Column 6: east velocity (m/s)
% Column 7: down velocity (m/s)
% Column 8: roll component of NED attitude error (deg)
% Column 9: pitch component of NED attitude error (deg)
% Column 10: yaw component of NED attitude error (deg)

% Begins

%%%%%%%%%%%%%%%%%%%%%%%%%%%%position
figure;
subplot(3,1,1);
set(gca,'NextPlot','replacechildren');

plot(errors1(:,1),(errors1(:,2)),'k','LineWidth',1.5);
hold on;
plot(errors2(:,1),(errors2(:,2)),'r','LineWidth',1.5);
title('Position error')
xlabel('Tims (s)');
ylabel('North (m)');
legend('CKF','VS-PGAF');

subplot(3,1,2);
set(gca,'NextPlot','replacechildren');
% plot(errors1(:,1),(errors1(:,3)),'y','LineWidth',1.5);
% hold on;			     
plot(errors1(:,1),(errors1(:,3)),'k','LineWidth',1.5);
hold on;			        
plot(errors2(:,1),(errors2(:,3)),'r','LineWidth',1.5);
xlabel('Tims (s)');
ylabel('East (m)');

subplot(3,1,3);
set(gca,'NextPlot','replacechildren');		     
plot(errors1(:,1),(errors1(:,4)),'k','LineWidth',1.5);
hold on;			     
plot(errors2(:,1),(errors2(:,4)),'r','LineWidth',1.5);
xlabel('Tims(s)');
ylabel('Down (m)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%Velocity
figure;
subplot(3,1,1);
set(gca,'NextPlot','replacechildren');
plot(errors1(:,1),(errors1(:,5)),'k','LineWidth',1.5);
hold on;			     		     
plot(errors2(:,1),(errors2(:,5)),'r','LineWidth',1.5);
title('Velocity error');
xlabel('Tims (s)');
ylabel('North (m/s)');
legend('CKF','VS-PGAF');

subplot(3,1,2);
set(gca,'NextPlot','replacechildren');		     
plot(errors1(:,1),(errors1(:,6)),'k','LineWidth',1.5);
hold on;			     		     
plot(errors2(:,1),(errors2(:,6)),'r','LineWidth',1.5);
xlabel('Tims(s)');
ylabel('East (m/s)');

subplot(3,1,3);
set(gca,'NextPlot','replacechildren');		     
plot(errors1(:,1),(errors1(:,7)),'k','LineWidth',1.5);
hold on;			     		     
plot(errors2(:,1),(errors2(:,7)),'r','LineWidth',1.5);
xlabel('Tims(s)');
ylabel('Down (m/s)');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%Attitude
figure;
subplot(3,1,1);
set(gca,'NextPlot','replacechildren');	     
plot(errors1(:,1),radtodeg(errors1(:,8)),'k','LineWidth',1.5);
hold on;				     			     
plot(errors2(:,1),radtodeg(errors2(:,8)),'r','LineWidth',1.5);
title('Attitude error');
xlabel('Tims(s)');
ylabel('North (^o)');
legend('CKF','VS-PGAF');

subplot(3,1,2);
set(gca,'NextPlot','replacechildren');			     
plot(errors1(:,1),radtodeg(errors1(:,9)),'k','LineWidth',1.5);
hold on;				     			     
plot(errors2(:,1),radtodeg(errors2(:,9)),'r','LineWidth',1.5);
xlabel('Tims (s)');
ylabel('East (^o)');

subplot(3,1,3);
set(gca,'NextPlot','replacechildren');
plot(errors1(:,1),radtodeg(errors1(:,10)),'k','LineWidth',1.5);
hold on;				      			      
plot(errors2(:,1),radtodeg(errors2(:,10)),'r','LineWidth',1.5);
xlabel('Tims (s)');
ylabel('Heading (^o)');

% Ends