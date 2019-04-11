function [] = myStopFcn2sim(xEst,yEst,X_array,Y_array,rp1,rp2)
mot_r = rospublisher('/pwm_cmd_r','std_msgs/Int16');
msg_r = rosmessage(mot_r);
msg_r.Data = 0;
mot_l = rospublisher('/pwm_cmd_l','std_msgs/Int16');
msg_l = rosmessage(mot_l);
msg_l.Data = 0;
pause(3);
send(mot_r,msg_r);
send(mot_l,msg_l);

figure('units','normalized','position',[.1 .1 .75 .75])
plot(yEst.signals.values(:,1),xEst.signals.values(:,1),'b-')
hold on
% plot(yEst.signals.values(:,2),xEst.signals.values(:,2),'b--')
legend('Actual Path');%,'Estimated Path')
plot(Y_array,X_array,'*r')
grid
vAxis = axis;

for ii = 1:length(X_array)
    my_circle(rp2,Y_array(ii),X_array(ii),1,1,'-r',1)
    my_circle(rp1,Y_array(ii),X_array(ii),1,1,'--b',1)
end
xlabel('y [m]')
ylabel('x [m]')
title('red stars indicate wayPoints')
axis equal
axis([vAxis(1)-2*max(rp1,rp2) vAxis(2)+2*max(rp1,rp2) vAxis(3)-2*max(rp1,rp2) vAxis(4)+2*max(rp1,rp2)])

hold off

function my_circle(r,x_c,y_c,x_scaling,h_scaling,lt,lw)
%
% draws a circle with radius r, centered at (x_c,y_c), linetype lt, and line
% width lw
%
angle=-pi:.01:pi;
for i = 1:length(angle)
    R=[x_scaling*cos(angle(i)),-x_scaling*sin(angle(i));...
        h_scaling*sin(angle(i)),h_scaling*cos(angle(i))];    
    temp=R*[r;0];
    x_circle(i)=temp(1);
    y_circle(i)=temp(2);
end
X_circle=x_c+x_circle;
Y_circle=y_c+y_circle;
plot(X_circle,Y_circle,lt,'LineWidth',lw)
