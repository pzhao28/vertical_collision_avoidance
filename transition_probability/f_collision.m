function [futureS]=f_collision(S,V,a,P,n)
%  Calculates the future state given:
%  Initial state S: 3*n matrix
%  Initial speed V: 2*(n+1) matrix [magnitude (knots); course angle (clockwise from north)]
%  Action a: rate of climb (ft/s) for reference aircraft
%  Collision point P: n*2 matrix (lon,lat)
%  # of intruders n: integer

h0=350; % 100s of feet, initial alt
h0_dot=0; % feet per sec, initial rate of climb

% needs discussion
h = S(1,:);  % inital relative height, ft
h_dot = S(2,:); % initial relative rate of climb 
H = [h0*100, h+h0*100];  % actual
H_dot = [h0_dot, h0_dot+h_dot];  % actual
tau = S(3,:);  % initia time to collision: sec

% calculate original point (initial p)
ip=zeros(n+1,2);
s=V(1,1)*tau(1)/3600;  % unit: nautical miles
ip(1,1)=P(1,1)-nm2deg(s*sin(V(2,1)));
ip(1,2)=P(1,2)-nm2deg(s*cos(V(2,1)));
for i=1:n
    s=V(1,i+1)*tau(i)/3600;  % unit: nautical miles
    ip(i+1,1)=P(i,1)-nm2deg(s*sin(V(2,i+1)));
    ip(i+1,2)=P(i,2)-nm2deg(s*cos(V(2,i+1)));
end

% initial ac
% load trx
environmentInterface.load_rap('share/tg/rap');
aircraftInterface.load_aircraft('share/tg/trx/flight3.trx', ... 
                                'share/tg/trx/flight3_mfl.trx');

aclist=aircraftInterface.getAllAircraftId();
noa=length(aclist);

if noa~=n+1
    error('incorrect trx input');
end

for kk=1:noa
    ac=aircraftInterface.select_aircraft(aclist(kk));
    
    ac.setLongitude_deg(ip(kk,1));
    ac.setLatitude_deg(ip(kk,2));
    ac.setAltitude_ft(H(kk));
    
    setTarget_waypoint_longitude_deg(P(kk,2));
    setTarget_waypoint_latitude_deg(P(kk,2));
    
    if kk==1
        ac.setRocd_fps(a);
    else
        ac.setRocd_fps(H_dot(kk));
    end
    
end

simulationInterface.setupSimulation(10, 1);
simulationInterface.start(1);  % pause at 1 second

%while loop to check simulation status / not important
while true
    server_runtime_sim_status = simulationInterface.get_runtime_sim_status();
     if (server_runtime_sim_status ~= NATS_SIMULATION_STATUS_PAUSE)
%         pause(1);
    else
        break;
    end
end

fp=zeros(size(ip));  % final position
fh=zeros(n+1,1);  % final height
fh_dot=fh;
for kk=1:noa
    ac=aircraftInterface.select_aircraft(aclist(kk));
    
    fp(kk,1)=ac.getLongitude_deg();
    fp(kk,2)=ac.getLatitude_deg();
    fh(kk)=ac.getAltitude_ft();
    fh_dot(kk)=ac.getRocd_fps();
end
    
simulationInterface.stop();

taup=zeros(size(tau));

for i=1:n
    
    dx=abs(fp(i+1,1)-P(i,1));
    dy=abs(fp(i+1,2)-P(i,2));
    s=sqrt(deg2nm(dx)^2+deg2nm(dy)^2);
    
    taup(i)=3600*s/V(1,i+1);
end

rh=fh-fh(1);  % new relative height
rh_dot=fh_dot-fhdot(1);  % relative roc
rh(1)=[];  % delete self
rh_dot(1)=[];  % delete self

futureS=[rh;rh_dot;taup];





    


















