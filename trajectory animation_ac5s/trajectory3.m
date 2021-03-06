function [M]=trajectory3(x,y,z,pitch,roll,yaw,scale_factor,step,selector,varargin);

%   function trajectory2(x,y,z,pitch,roll,yaw,scale_factor,step,[selector,SoR])
%   
%
%   x,y,z               center trajectory (vector)    [m]
%   
%   pitch,roll,yaw      euler's angles                [rad]
%   
%   scale_factor        normalization factor          [scalar]
%                              (related to body aircraft dimension)
%   
%   step                attitude sampling factor      [scalar]
%                              (the points number between two body models)
%   
%   selector            select the body model         [string]
%
%                       gripen  JAS 39 Gripen            heli        Helicopter
%                       mig     Mig			             ah64        Apache helicopter
%                       tomcat  Tomcat(Default)          a10
%                       jet     Generic jet		         cessna      Cessna
%                       747     Boeing 747		         biplane     Generic biplane
%                       md90    MD90 jet		         shuttle     Space shuttle
%                       dc10    DC-10 jet
%
%       OPTIONAL INPUT:
%
%
%    View               sets the camera view. Use Matlab's "view" as argument to reuse the current view.                    
%       
%       Note:
%
%    Refernce System:
%                       X body- The axial force along the X body  axis is
%                       positive along forward; the momentum around X body
%                       is positive roll clockwise as viewed from behind;
%                       Y body- The side force along the Y body axis is
%                       positive along the right wing; the moment around Y
%                       body is positive in pitch up;
%                       Z body- The normal force along the Z body axi30000s is
%                       positive down; the moment around Z body is positive
%                       roll clockwise as viewed from above.
%
%   *******************************
%   Function Version 3.0 
%   7/08/2004 (dd/mm/yyyy)
%   Valerio Scordamaglia
%   v.scordamaglia@tiscali.it
%   *******************************

% if nargin<9
%     disp('  Error:');
% 
%     disp('      Error: Invalid Number Inputs!');
%     M=0;
%     return;
% end
% if (length(x)~=length(y))|License checkout failed.
% License Manager Error -96(length(x)~=length(z))|(length(y)~=length(z))
%     disp('  Error:');
%     disp('      Uncorrect Dimension of the center trajectory Vectors. Please Check the size');
%     M=0;
%     return;
% end
% if ((length(pitch)~=length(roll))||(length(pitch)~=length(yaw))||(length(roll)~=length(yaw)))
%     disp('  Error:');
%     disp('      Uncorrect Dimension of the euler''s angle Vectors. Please Check the size');
% M=0;
%     return;
% end
% if length(pitch)~=length(x)
%     disp('  Error:');
%     disp('      Size mismatch between euler''s angle vectors and center trajectory vectors');
%     M=0;
%     return
% end
% if step>=length(x)
%     disp('  Error:');
%     disp('      Attitude samplig factor out of range. Reduce step');
% M=0;
%     return
% end
% if step<1
%     step=1;
% 
% end
% 
% if nargin==10
%     
%     theView=cell2mat(varargin(1));
% 
% end
% if nargin>10
%     disp('Too many inputs arguments');
%     M=0;
%     return
% end
if nargin<10

%     theView=[82.50 2];
    theView=[50 30];
end




mov=nargout;

cur_dir=pwd;

if strcmp(selector,'shuttle')
    load shuttle;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'helicopter')
    load helicopter;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'747')
    load boeing_747;
    V=[V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'biplane')
    load biplane;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'md90')
    load md90;
    V=[-V(:,1) V(:,2) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'dc10')
    load dc10;
    V=[V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'ah64')
    load ah64;
    V=[V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'mig')
    load mig;
    V=[V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'tomcat')
    load tomcat;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'jet')
    load 80jet;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'cessna')
    load 83plane;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'A-10')
    load A-10;
    V=[V(:,3) V(:,1) V(:,2)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'gripen')
    load gripen;
    V=[-V(:,1) -V(:,2) V(:,3)];

    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
else
    
    try
    eval(['load ' selector ';']);
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
    catch
    str=strcat('Warning: ',selector,' not found.    Default=A-10');
    disp(str);
    load A-10;
    V=[V(:,3) V(:,1) V(:,2)];
    end

end
correction=max(abs(V(:,1)));
V=V./(scale_factor*correction);
V1=V;
for i=1:length(V)
    V(i,:)=[[cos(1.5153) -sin(1.5153) 0 ; sin(1.5153) cos(1.5153) 0;0 0 1]*V(i,:)']';
end
V2=V;
V3=V;
V4=V;
V5=V;
ii=length(x);
resto=mod(ii,step);
%%%%%%%%%%%%%%%needed for the transformation%%%%%%%%%%%%%%%
x1=x(:,1);
x2=x(:,2);
x3=x(:,3);
x4=x(:,4);
x5=x(:,5);
y1=y(:,1);
y2=y(:,2);
y3=y(:,3);
y4=y(:,4);
y5=y(:,5);
z1=z(:,1);
z2=z(:,2);
z3=z(:,3);
z4=z(:,4);
z5=z(:,5);


    pitch1=pitch(:,1);
    pitch2=pitch(:,2);
    pitch3=pitch(:,3);
    pitch4=pitch(:,4);
    pitch5=pitch(:,5);
    roll1=roll(:,1);
    roll2=roll(:,2);
    roll3=roll(:,3);
    roll4=roll(:,4);
    roll5=roll(:,5);
    yaw1=-yaw(:,1);
    yaw2=-yaw(:,2);
    yaw3=-yaw(:,3);
    yaw4=-yaw(:,4);
    yaw5=-yaw(:,5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
frame = 0;
for i=1:step:(ii-resto)
    
if mov | (i == 1)
      clf;
      plot3(x1,y1,z1,'r-');
      hold on;
       plot3(x2,y2,z2,'g-');
       hold on;
       plot3(x3,y3,z3,'b-');
       hold on;
       plot3(x4,y4,z4,'y-')
       hold on;
       plot3(x5,y5,z5,'c-')
%        set(gca,'XLim',[0 14e4], 'YLim', [-7000 18e4],'ZLim',[2500 3500]);
      grid on;
      hold on;
      light;
    end
theta1=pitch1(i);
theta2=pitch2(i);
theta3=pitch3(i);
theta4=pitch4(i);
theta5=pitch5(i);
phi1=-roll1(i);
phi2=-roll2(i);
phi3=-roll3(i);
phi4=-roll4(i);
phi5=-roll5(i);
psi1=yaw1(i);
psi2=yaw2(i);
psi3=yaw3(i);
psi4=yaw4(i);
psi5=yaw5(i);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Tbe1=[cos(psi1)*cos(theta1), -sin(psi1)*cos(theta1), sin(theta1);
	 cos(psi1)*sin(theta1)*sin(phi1)+sin(psi1)*cos(phi1) ...
	 -sin(psi1)*sin(theta1)*sin(phi1)+cos(psi1)*cos(phi1) ...
	 -cos(theta1)*sin(phi1);
	 -cos(psi1)*sin(theta1)*cos(phi1)+sin(psi1)*sin(phi1) ...
	 sin(psi1)*sin(theta1)*cos(phi1)+cos(psi1)*sin(phi1) ...
	 cos(theta1)*cos(phi1)];
 
 Tbe2=[cos(psi2)*cos(theta2), -sin(psi2)*cos(theta2), sin(theta2);
	 cos(psi2)*sin(theta2)*sin(phi2)+sin(psi2)*cos(phi2) ...
	 -sin(psi2)*sin(theta2)*sin(phi2)+cos(psi2)*cos(phi2) ...
	 -cos(theta2)*sin(phi2);
	 -cos(psi2)*sin(theta2)*cos(phi2)+sin(psi2)*sin(phi2) ...
	 sin(psi2)*sin(theta2)*cos(phi2)+cos(psi2)*sin(phi2) ...
	 cos(theta2)*cos(phi2)];
 
  Tbe3=[cos(psi3)*cos(theta3), -sin(psi3)*cos(theta3), sin(theta3);
	 cos(psi3)*sin(theta3)*sin(phi3)+sin(psi3)*cos(phi3) ...
	 -sin(psi3)*sin(theta3)*sin(phi3)+cos(psi3)*cos(phi3) ...
	 -cos(theta3)*sin(phi3);
	 -cos(psi3)*sin(theta3)*cos(phi3)+sin(psi3)*sin(phi3) ...
	 sin(psi3)*sin(theta3)*cos(phi3)+cos(psi3)*sin(phi3) ...
	 cos(theta3)*cos(phi3)];
 
  Tbe4=[cos(psi4)*cos(theta4), -sin(psi4)*cos(theta4), sin(theta4);
	 cos(psi4)*sin(theta4)*sin(phi4)+sin(psi4)*cos(phi4) ...
	 -sin(psi4)*sin(theta4)*sin(phi4)+cos(psi4)*cos(phi4) ...
	 -cos(theta4)*sin(phi4);
	 -cos(psi4)*sin(theta4)*cos(phi4)+sin(psi4)*sin(phi4) ...
	 sin(psi4)*sin(theta4)*cos(phi4)+cos(psi4)*sin(phi4) ...
	 cos(theta4)*cos(phi4)];
 
  Tbe5=[cos(psi5)*cos(theta5), -sin(psi5)*cos(theta5), sin(theta5);
	 cos(psi5)*sin(theta5)*sin(phi5)+sin(psi5)*cos(phi5) ...
	 -sin(psi5)*sin(theta5)*sin(phi5)+cos(psi5)*cos(phi5) ...
	 -cos(theta5)*sin(phi5);
	 -cos(psi5)*sin(theta5)*cos(phi5)+sin(psi5)*sin(phi5) ...
	 sin(psi5)*sin(theta5)*cos(phi5)+cos(psi5)*sin(phi5) ...
	 cos(theta5)*cos(phi5)];
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vnew1=V1*Tbe1;
Vnew2=V2*Tbe2;
Vnew3=V3*Tbe3;
Vnew4=V4*Tbe4;
Vnew5=V5*Tbe5;
rif1=[x1(i) y1(i) z1(i)];
rif2=[x2(i) y2(i) z2(i)];
rif3=[x3(i) y3(i) z3(i)];
rif4=[x4(i) y4(i) z4(i)];
rif5=[x5(i) y5(i) z5(i)];

X01=repmat(rif1,size(Vnew1,1),1);
X02=repmat(rif2,size(Vnew2,1),1);
X03=repmat(rif3,size(Vnew3,1),1);
X04=repmat(rif4,size(Vnew4,1),1);
X05=repmat(rif5,size(Vnew5,1),1);

Vnew1=Vnew1+X01;
Vnew2=Vnew2+X02;
Vnew3=Vnew3+X03;
Vnew4=Vnew4+X04;
Vnew5=Vnew5+X05;

p1=patch('faces', F, 'vertices' ,Vnew1);
set(p1, 'facec', [1 0 0]);          
set(p1, 'EdgeColor','r'); 


p2=patch('faces', F, 'vertices' ,Vnew2);
set(p2, 'facec', [0 0 1]);          
set(p2, 'EdgeColor','g'); 

p3=patch('faces', F, 'vertices' ,Vnew3);
set(p3, 'facec', [0 0 1]);          
set(p3, 'EdgeColor','b'); 

p4=patch('faces', F, 'vertices' ,Vnew4);
set(p4, 'facec', [0 0 1]);          
set(p4, 'EdgeColor','y'); 

p5=patch('faces', F, 'vertices' ,Vnew5);
set(p5, 'facec', [0 0 1]);          
set(p5, 'EdgeColor','c'); 

if mov | (i == 1)
     view(theView);
       
         set(gca,'XLim',[-100 100], 'YLim', [-100 100],'ZLim',[0 400]);
          %axis equal;
%         axis square;
 xlabel('X');
ylabel('Y');
zlabel('Z');
view(45,15)
       %set(gca,'ZLim',[0 200]);
end
if mov
if i == 1
	ax = axis;
else
	axis(ax);
      end
      lighting phong
      frame = frame + 1;
      M(frame) = getframe;
    end

end
% hold on;
% plot3(x,y,z);
% lighting phong;
% grid on;
% view(theView);
% 
% daspect([1 1 1]) ;
% 
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% % set(gca,'XTick',0:20:100);0
% % set(gca,'YTick',0:200:1000);
% % set(gca,'ZTick',0:200:1000);
% set(gca,'XLim',[0 14e4], 'YLim', [-7000 18e4],'ZLim',[3000 3100]);
cd (cur_dir);























