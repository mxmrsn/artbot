clear all; close all; clc;
%% script to test art robot kinematics
canvas_dims = [36*25.4 72*25.4]; % w x h
canvas_offsets = [300 100]; % w; h

c0 = [0+canvas_offsets(1); 0-canvas_offsets(2)];  % left-upper
c1 = [c0(1) + canvas_dims(1); c0(2)];             % right-upper
c2 = [c1(1); c1(2) - canvas_dims(2)];             % right-lower
c3 = [c0(1); c0(2) - canvas_dims(2)];             % left-lower

ell0 = [800 800]; % initial lengths
motor_spacing = 1500; % mm
pulley_rad = 20; % mm

q = [15 10]; % joint values (rad) % use to solve forward kinematics

params.canvas_bnds = [c0';c1';c2';c3'];
params.canvas_dims = canvas_dims;
params.canvas_offsets = canvas_offsets;
params.pulley_rad = pulley_rad;
params.ell0 = ell0;
params.motor_spacing = motor_spacing;

h1 = figure(1); h1.Color = 'w'; clf;
plotRobot(q,params);

% J = numericalJacobian(q,params);

[ptip_Mfkin,theta_fkin] = fkin(q,params);   % test fkin
[q2,theta2] = ikin(ptip_Mfkin,params);      % test ikin

% compute ik joint limits based on canvas corners
for ii = 1:length(params.canvas_bnds)
    clf;
    [q_bnds(ii,:),~] = ikin(params.canvas_bnds(ii,:),params);
    ell_bnds(ii,:) = q2ell(q_bnds,params);
    
    plotRobot(q_bnds(ii,:),params);
    drawnow();
    pause(0.2);
end

% %% inform min/max belt lengths
ell_min = min(ell_bnds);
ell_max = max(ell_bnds);
disp(['Min Belt Lengths: ',num2str(ell_min),' | Max Belt Lengths: ',num2str(ell_max)]);

%% generate joint space configs
qmin = min(q_bnds); qmax = max(q_bnds);
q1range = linspace(qmin(1),qmax(1),30); q2range = linspace(qmin(2),qmax(2),30);
count = 1;
for ii = 1:length(q1range)
    for jj = 1:length(q2range)
        qij = [q1range(ii) q2range(jj)];
        [ptip_fk(:,count),~] = fkin(qij,params);
        Jtip_fk(:,:,count) = numericalJacobian(qij,params);
        [~,s,~] = svd(Jtip_fk(:,:,count));
        svtip(count) = s(1,1).*s(2,2); % product of singular values as dexterity metric
        count = count + 1;
    end
end
[k,~] = boundary(ptip_fk');
plot(ptip_fk(1,k),ptip_fk(2,k),'Color','c','LineWidth',2);

% scatter(ptip_fk(1,:),ptip_fk(2,:),'filled','MarkerEdgeColor','k');
svmax = max(svtip);
svint = svtip./svmax;
scatter(ptip_fk(1,:),ptip_fk(2,:),20,svint,'filled','MarkerEdgeColor','k');
colorbar;

%% generate points in task space
pmin = min(params.canvas_bnds); pmax = max(params.canvas_bnds);
xrange = linspace(pmin(1),pmax(1),20); yrange = linspace(pmin(2),pmax(2),20);
count = 1;
for ii = 1:length(xrange)
    for jj = 1:length(yrange)
        [qtest(:,count),~] = ikin([xrange(ii) yrange(jj)],params);
        ptip_ik(:,count) = fkin(qtest(:,count),params);
        Jtip_ik(:,:,count) = numericalJacobian(qtest(:,count),params);
        [~,si,~] = svd(Jtip_ik(:,:,count));
        svtip_ik(count) = si(1,1).*si(2,2);
        count = count + 1;
    end
end
h2 = figure(2); h2.Color = 'w'; clf;
plotRobot([0 0],params); % plot robot at home position

% scatter(ptip_ik(1,:),ptip_ik(2,:),'filled','MarkerEdgeColor','k');
svikmax = max(svtip_ik);
svikint = svtip_ik./svikmax;
scatter(ptip_ik(1,:),ptip_ik(2,:),20,svikint,'filled','MarkerEdgeColor','k');
colorbar;

%% Helper Functions
function plotRobot(q,params)

    ptip_M = fkin(q,params);

%     clf;
    
    % plot canvas
    P = params.canvas_bnds;
    F0 = [1 2 3 4];
    patch('Faces',F0,'Vertices',P,'FaceColor','b','FaceAlpha',0.4,'EdgeColor','b','LineWidth',2);   % patch canvas
    
    % plot angles
    p1 = [0; 0];
    p2 = [params.motor_spacing; 0];
    p3 = [ptip_M(1); 0];
    P1 = [p1';p3';ptip_M'];
    P2 = [p2';ptip_M';p3'];
    F = [1 2 3];
    patch('Faces',F,'Vertices',P1,'FaceColor','c','FaceAlpha',0.5,'EdgeAlpha',0);   % patch angles
    patch('Faces',F,'Vertices',P2,'FaceColor','m','FaceAlpha',0.5,'EdgeAlpha',0);
    
    % plot motors, EE, and pulley belts
    line([0 params.motor_spacing],[0 0],'LineWidth',2,'Color','k'); hold on; grid on; grid minor;                   % draw motor mount
    line([0 ptip_M(1)],[0 ptip_M(2)],'Color','k','LineWidth',2);                                  % draw left pulley belt
    line([params.motor_spacing ptip_M(1)],[0 ptip_M(2)],'Color','k','LineWidth',2);               % draw right pulley belt
    
    scatter([0 params.motor_spacing],[0 0],100,'filled','MarkerFaceColor','y','MarkerEdgeColor','k','LineWidth',2); % draw motors
    scatter(ptip_M(1),ptip_M(2),100,'filled','MarkerFaceColor','r','MarkerEdgeColor','k','LineWidth',2);            % draw end effector
    
    % plot coord frames
    sc = 50;
    quiver(0,0,sc*1,sc*0,'r','LineWidth',2);  % Mx
    quiver(0,0,sc*0,sc*-1,'g','LineWidth',2); % My
    quiver(params.canvas_offsets(1),-params.canvas_offsets(2),sc*1,sc*0,'r','LineWidth',2);  % Cx
    quiver(params.canvas_offsets(1),-params.canvas_offsets(2),sc*0,sc*-1,'g','LineWidth',2); % Cy
    
    axis equal
end

function [ptip_M, theta] = fkin(q,params)

    % TODO: incorporate pulley_rad in motor_spacing
    % TODO: incorporate angle effect on belt origin at pulley

    ell = q2ell(q,params);
    ell1 = ell(1); ell2 = ell(2);
    ell3 = params.motor_spacing;

    % solve for these with law of cosines
    theta1 = real( acosd( -(ell2^2 - ell1^2 - ell3^2) / (2*ell1*ell3) ) );
    theta2 = real( acosd( -(ell1^2 - ell2^2 - ell3^2) / (2*ell2*ell3) ) );
    theta3 = real( acosd( -(ell3^2 - ell1^2 - ell2^2) / (2*ell1*ell2) ) );

    theta = [theta1; theta2; theta3];
%     if (sum(theta) ~= 180)
%         disp(['Kinematics Error: triangle constraint violated! | sum(theta): ',num2str(sum(theta))]);
%     end
    ptip_M = [ell1*cosd(theta1); -ell1*sind(theta1)];
    
end
function [q, theta] = ikin(ptip_M,params)

    px = ptip_M(1); py = abs(ptip_M(2));
    
    ell1 = sqrt(px^2 + py^2);
    theta1 = atan2d(py,px);
    
    px2 = (params.motor_spacing - px); py2 = py;
    ell2 = sqrt(px2^2 + py2^2);
    theta2 = atan2d(py2,px2);

    q = ell2q([ell1 ell2],params);
    theta = [theta1; theta2];

end
function J = numericalJacobian(q,params)

    d_ell = 0.01;
    for ii = 1:length(q)
        
        dir = zeros(size(q));
        dir(ii) = 1;
        
        q_up = q + d_ell*dir;
        q_down = q -d_ell*dir;
        
        ptip_up = fkin(q_up,params);
        ptip_down = fkin(q_down,params);
        
        J(ii,:) = (ptip_up-ptip_down)'./(2*d_ell);
    end
    
end

function q = ell2q(ell,params)

    ell1 = ell(1); ell2 = ell(2);
    q(1) = ( ell1 - params.ell0(1) ) / params.pulley_rad;
    q(2) = ( ell2 - params.ell0(2) ) / params.pulley_rad;

end
function ell = q2ell(q,params)

    ell1 = params.ell0(1) + params.pulley_rad*q(1);
    ell2 = params.ell0(2) + params.pulley_rad*q(2);
    ell = [ell1 ell2];
end