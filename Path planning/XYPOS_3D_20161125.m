% clc;clear all;close;
%     
%     [angle1,x1,y1,time]=textread('control_pos_m.txt','%f%f%f%f','headerlines',0);
%  %   angle1=angle1+pi/2;
% %     x1=x1*100;
% %     y1=y1*100;
%     time=time/1000;
%     
%    halfL = 0.21/2; halfW =0.21/2;     %%%   original robot posture (corner)
%    cornerR = [halfL -halfL -halfL halfL halfL;
%    halfW halfW -halfW -halfW halfW];
%    arrowR = [1.5*halfL 0]';  % robot orientation arrow
% 
%     for i=0:5:size(x1,1)
%        x2(1+ i/5)=x1(i+1);
%        y2(1+ i/5)=y1(i+1);
%        time2(1+ i/5)=time(i+1);
%     end
%     plot3(x2,y2,time2,'r','LineWidth',2);hold on;
% %     plot(x1,y1,'r','LineWidth',1);hold on;
%     
%    for i=1:10:size(x1,1)
%        %%if((((x(i+1)-x(i))^2+y(i+1)-y(i))^2) < 6) 
%         
%         
%            angle1(i) = wrapToPi( angle1(i) );
%            rotationM = [cos(angle1(i))  -sin(angle1(i)) ; sin(angle1(i))  cos(angle1(i))];
%            rotated_cornerR = rotationM * cornerR + [x1(i) 0;0 y1(i)]*ones(size(cornerR));
%            rotated_arrowR = rotationM * arrowR + [x1(i) y1(i)]';    
%            time_vector = [time(i) time(i) time(i) time(i) time(i)];
%            plot3(rotated_cornerR(1,:), rotated_cornerR(2,:),time_vector,'g-','LineWidth',2.5); hold on;
%            plot3([x1(i) rotated_arrowR(1)],[y1(i) rotated_arrowR(2)],[time(i) time(i)],'b-','LineWidth',1.5); hold on;
%            if i==1;
%                 plot3(rotated_cornerR(1,:), rotated_cornerR(2,:),time_vector,'r-','LineWidth',2.5); hold on;
%                 plot3([x1(i) rotated_arrowR(1)],[y1(i) rotated_arrowR(2)],[time(i) time(i)],'r-','LineWidth',1.5); hold on;
%            end
%       %% end
%    end
%   angle1(size(x1,1)) = wrapToPi( angle1(size(x1,1)) );
%   rotationM = [cos(angle1(size(x1,1)))  -sin(angle1(size(x1,1))) ; sin(angle1(size(x1,1)))  cos(angle1(size(x1,1)))];
%   rotated_cornerR = rotationM * cornerR + [x1(size(x1,1)) 0;0 y1(size(x1,1))]*ones(size(cornerR));
%   rotated_arrowR = rotationM * arrowR + [x1(size(x1,1)) y1(size(x1,1))]';  
%   time_vector = [time(size(x1,1)) time(size(x1,1)) time(size(x1,1)) time(size(x1,1)) time(size(x1,1))];
%   plot3(rotated_cornerR(1,:), rotated_cornerR(2,:),time_vector,'--black','LineWidth',2.5); hold on;
%   plot3([x1(size(x1,1)) rotated_arrowR(1)],[y1(size(x1,1)) rotated_arrowR(2)],[time(size(x1,1)) time(size(x1,1))],'--black','LineWidth',1.5); hold on;
%   
% title('\fontsize{14} \fontname{Times New Roman} trajectory');
% xlabel('\fontsize{16} \fontname{Times New Roman} m');
% ylabel('\fontsize{16} \fontname{Times New Roman} m');
% zlabel('\fontsize{16} \fontname{Times New Roman} ms');
%  % axis([-0.5,2.5,-2.5,0.5]);grid on;
% grid on;
%  axis equal;

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  clc;clear all;close;
    
    [angle1,x1,y1,time]=textread('control_pos_m22.txt','%f%f%f%f','headerlines',0);
    [xx,yy]=textread('control_pos_m_theorem.txt','%f%f','headerlines',0);
 %   angle1=angle1+pi/2;
%     x1=x1*100;
%     y1=y1*100;
    time=time/1000;
    
   halfL = 0.21/2; halfW =0.21/2;     %%%   original robot posture (corner)
   cornerR = [halfL -halfL -halfL halfL halfL;
   halfW halfW -halfW -halfW halfW];
   wheel_corner = [0.02 -0.06   0.02  -0.06;
                  halfW halfW -halfW -halfW;];
   arrowR = [1.5*halfL 0]';  % robot orientation arrow

    for i=0:5:size(x1,1)-1
       x2(1+ i/5)=x1(i+1);
       y2(1+ i/5)=y1(i+1);
    end
%     plot(x2,y2,'r','LineWidth',2);hold on;
%     plot(x1,y1,'r','LineWidth',1);hold on;
    
   for i=1:10:size(x1,1)
       %%if((((x(i+1)-x(i))^2+y(i+1)-y(i))^2) < 6) 
        
        
           angle1(i) = wrapToPi( angle1(i) );
           rotationM = [cos(angle1(i))  -sin(angle1(i)) ; sin(angle1(i))  cos(angle1(i))];
           rotated_cornerR = rotationM * cornerR + [x1(i) 0;0 y1(i)]*ones(size(cornerR));
           wheel_cornerR = rotationM * wheel_corner + [x1(i) 0;0 y1(i)]*ones(size(wheel_corner));
           rotated_arrowR = rotationM * arrowR + [x1(i) y1(i)]';       
           %車體填滿
           fill(rotated_cornerR(1,:)',rotated_cornerR(2,:)',[1 0.7 0.45]); hold on;
           %車體外框
%            plot(rotated_cornerR(1,:), rotated_cornerR(2,:),'r-','LineWidth',1); hold on;
           %左右輪子
           plot(wheel_cornerR(1,1:2), wheel_cornerR(2,1:2),'k-','LineWidth',4); hold on;
           plot(wheel_cornerR(1,3:4), wheel_cornerR(2,3:4),'k-','LineWidth',4); hold on;
           %朝向符號
%            plot([x1(i) rotated_arrowR(1)],[y1(i) rotated_arrowR(2)],'b-','LineWidth',1.5); hold on;
           %起始位置
%            if i==1;
%                 plot(rotated_cornerR(1,:), rotated_cornerR(2,:),'r-','LineWidth',2.5); hold on;
%                 plot([x1(i) rotated_arrowR(1)],[y1(i) rotated_arrowR(2)],'r-','LineWidth',1.5); hold on;
%            end
      %% end
   end
   
  %終點位置
  angle1(size(x1,1)) = wrapToPi( angle1(size(x1,1)) );
  rotationM = [cos(angle1(size(x1,1)))  -sin(angle1(size(x1,1))) ; sin(angle1(size(x1,1)))  cos(angle1(size(x1,1)))];
  rotated_cornerR = rotationM * cornerR + [x1(size(x1,1)) 0;0 y1(size(x1,1))]*ones(size(cornerR));
  rotated_arrowR = rotationM * arrowR + [x1(size(x1,1)) y1(size(x1,1))]';  
  plot(rotated_cornerR(1,:), rotated_cornerR(2,:),'-r','LineWidth',3); hold on;
  plot([x1(size(x1,1)) rotated_arrowR(1)],[y1(size(x1,1)) rotated_arrowR(2)],'-r','LineWidth',3); hold on;
  %模擬的路徑
  h1 = plot(xx,yy,'-.k','LineWidth',2.5);hold on;
  %實際行走的路徑
  h2 = plot(x2,y2,'b','LineWidth',2.5);hold on;
  h3 = legend([h1 h2] ,'simulation','experiment',4);
  set(h3,'fontname','Times New Roman','fontsize',25);
  
title('trajectory','fontname','Times New Roman','fontsize',20,'fontweight','bold');
xlabel('x(m)','fontname','Times New Roman','fontsize',18,'fontweight','bold','fontangle','italic');
ylabel('y(m)','fontname','Times New Roman','fontsize',18,'fontweight','bold','fontangle','italic');
set(gca,'fontname','Times New Roman','fontsize',18)
 % axis([-0.5,2.5,-2.5,0.5]);grid on;
grid on;
axis equal;

   