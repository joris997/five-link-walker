function createAnimation(states,time,stFoot,p)
video = true;
if video
    writerObj = VideoWriter('out.avi');
    writerObj.FrameRate = 60;
    open(writerObj)
end

%% Angular momentum
%     masses   = [p(7) p(6) p(5) p(6) p(7)];
%     inertias = [p(10) p(9) p(8) p(9) p(10)];
    masses   = [p(6) p(7) p(5) p(7) p(6)];
    inertias = [p(9) p(10) p(8) p(10) p(9)];
    
    AngMom = zeros(length(time),3);
    for i = 1:length(time)
        % relative to absolute orientations/ang velocities
        qL = q_to_qL(states(i,1:5)',states(i,6:10)');
%         phi = [qL(6) qL(8) qL(7) qL(9) qL(10)];
        phi = [qL(8) qL(6) qL(10) qL(7) qL(9)];

        xyJ  = [pJoints(states(i,1:5)',p); zeros(1,5)];
        xyB  = [pBodies(states(i,1:5)',p); zeros(1,5)];
        vxyJ = [vJoints(states(i,1:5)',states(i,6:10)',p); zeros(1,5)];
        vxyB = [vBodies(states(i,1:5)',states(i,6:10)',p); zeros(1,5)];

        H = 0;
        for b = 1:size(xyB,2)
            r = xyB(:,b);
            v = vxyB(:,b);
            
            H = H + cross(r,masses(b).*-v) + inertias(b)*phi(b);
        end
        AngMom(i,:) = H';
    end
    
    pCOM = zeros(length(time),2);
    vCOM = zeros(length(time),2);
    for i = 1:length(time)
        pCOM(i,:) = pCoM(states(i,1:5)',p);
        vCOM(i,:) = vCoM(states(i,1:5)',states(i,6:10)',p);
    end

%% Animation
    stepIdx = find(diff(stFoot)>0.0);
    step = 0;
    
    figure(1010)
    set(gcf, 'Position', get(0, 'Screensize'));
    
    timei = 0.010;
    for i = 1:length(time)
        if time(i) > timei
            try
                if i >= stepIdx(step+1)
                    step = step + 1;
                end
            catch
                step = step;
            end
            
            % walker
            xy = pJointse([states(i,1:5)';stFoot(i,:)'],p);
            xyB = pBodiese([states(i,1:5)';stFoot(i,:)'],p);
        
            figure(1010);
            subplot(3,5,[1 2 3 6 7 8]); cla; hold on;
            if mod(step,2) == 0
                plot([xy(1,1);xy(1,2)],[xy(2,1);xy(2,2)],'c','LineWidth',6)
                plot([xy(1,2);xy(1,3)],[xy(2,2);xy(2,3)],'b','LineWidth',6)

                plot([xy(1,3);xyB(1,3)],[xy(2,3);xyB(2,3)],'k','LineWidth',6)

                plot([xy(1,3);xy(1,4)],[xy(2,3);xy(2,4)],'g','LineWidth',6)
                plot([xy(1,4);xy(1,5)],[xy(2,4);xy(2,5)],'r','LineWidth',6)
            else
                plot([xy(1,1);xy(1,2)],[xy(2,1);xy(2,2)],'r','LineWidth',6)
                plot([xy(1,2);xy(1,3)],[xy(2,2);xy(2,3)],'g','LineWidth',6)

                plot([xy(1,3);xyB(1,3)],[xy(2,3);xyB(2,3)],'k','LineWidth',6)

                plot([xy(1,3);xy(1,4)],[xy(2,3);xy(2,4)],'b','LineWidth',6)
                plot([xy(1,4);xy(1,5)],[xy(2,4);xy(2,5)],'c','LineWidth',6)
            end
            yline(0)
            xlim([0.0 3.0])
            ylim([-0.5 1.5])
            
            % CoM velocity
            subplot(3,5,[4 5]); cla; hold on; grid on;
            plot(time(i),vCOM(i,1),'bo','MarkerSize',12,'MarkerFaceColor','b')
            plot(time(i),vCOM(i,2),'ro','MarkerSize',12,'MarkerFaceColor','r')
            plot(time(1:i),vCOM(1:i,1),'r','LineWidth',2)
            plot(time(1:i),vCOM(1:i,2),'b','LineWidth',2)
            xlabel('time [s]'); ylabel('CoM velocity [m/s]');
            title('CoM velocity as a function of time')
            legend('$\dot{x}_{CoM}$','$\dot{z}_{CoM}$','Interpreter','latex')
            xlim([0 max(time)])
            ylim([min([vCOM(:,1);vCOM(:,2)]) max([vCOM(:,1);vCOM(:,2)])])
            
            % angular momentum
            subplot(3,5,[9 10]); cla; hold on; grid on;
            plot(time(1:i),AngMom(1:i,3),'r','LineWidth',2)
            plot(time(i),AngMom(i,3),'bo','MarkerSize',12,'MarkerFaceColor','b')
            xlabel('time [s]'); ylabel('Momentum [kg m^2/s]')
            title('Angular Momentum around stance foot')
            xlim([0 max(time)])
            ylim([min(AngMom(:,3)) max(AngMom(:,3))])
            
            % states
            coords = {'st_{hip}','sw_{hip}','st_{knee}','sw_{knee}','pelvis'};
            for coord = 1:5
                subplot(3,5,10+coord); cla; hold on; grid on;
                plot(states(1:i,coord),states(1:i,coord+5),'b','LineWidth',2)
                plot(states(i,coord),states(i,coord+5),'ro','MarkerSize',12,'MarkerFaceColor','r')
                xlim([min(states(:,coord)) max(states(:,coord))])
                ylim([min(states(:,coord+5)) max(states(:,coord+5))])
                xlabel('$q$ [rad]','Interpreter','latex'); ylabel('$\dot{q}$ [rad/s]','Interpreter','latex')
                title(coords{coord})
            end            
            drawnow
            set(gca,'FontSize',12)
            
            timei = timei + 0.0167;
            
            if video
                frame = getframe(gcf);
                writeVideo(writerObj,frame);
            end
        end
    end
    
    hold off
    if video
        close(writerObj);
    end
end

