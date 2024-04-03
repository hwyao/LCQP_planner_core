function [f,t,aCell] = plotConfiguration(tableInput,distMax,highLightDist,prePadding,postPadding)
    % constants
    jointName = "/joint_states/panda_joint" + (1:7) + "/position";
    velocityName = "/joint_states/panda_joint" + (1:7) + "/velocity";
    effortName = "/joint_states/panda_joint" + (1:7) + "/effort";
    distanceName = "/LCQP_controller_multipleObstacle/distance/data." + (0:distMax);

    % file configurations
    f = figure("Position",[241.8,66.6,524.8,669.6]);
    t = tiledlayout(f,4,1);

    % configuration of the data
    tableInput = renamevars(tableInput,"__time","time");
    tMin = tableInput.time(1);
    tMax = tableInput.time(end);
    tMinUse = tMin + prePadding;
    tMaxUse = tMax - postPadding;

    table = tableInput(tableInput.time >= tMinUse & tableInput.time <= tMaxUse,:);
    table.time = table.time - table.time(1);
    tMinUse = table.time(1);
    tMaxUse = table.time(end);
    styleCell = {"LineWidth",1.5}; 
    
    % plot the result
    ax1 = nexttile(t);
    plotStacked(ax1,table,jointName,styleCell)
    grid(ax1,"on");
    xlabel(ax1,"");
    ylabel(ax1,"$q$ [rad]","Interpreter","latex");
    xlim(ax1,[tMinUse,tMaxUse]);
    legend(ax1,"joint "+(1:7));
    
    ax2 = nexttile(t);
    plotStacked(ax2,table,velocityName,styleCell)
    grid(ax2,"on");
    xlabel(ax2,"");
    ylabel(ax2,"$\dot{q}$ [rad/s]","Interpreter","latex");
    xlim(ax2,[tMinUse,tMaxUse]);

    ax3 = nexttile(t);
    plotStacked(ax3,table,effortName,styleCell)
    grid(ax3,"on");
    xlabel(ax3,"");
    ylabel(ax3,"$\tau$ [Nm]","Interpreter","latex");
    xlim(ax3,[tMinUse,tMaxUse]);

    ax4 = nexttile(t);
    grid(ax4,"on");
    table = table(:,["time",distanceName]);
    table = rmmissing(table);
    ax4.NextPlot = "add";
    
    lName = flip(unique([highLightDist+1,(1:distMax+1)]));
    for iName = lName
        h = plot(ax4,table,"time",distanceName(iName),styleCell{:});
        if ~ismember(iName,highLightDist+1)
            h.Color = [0.5,0.5,0.5,0.5];
            h.LineWidth =  1.0;
        end
    end
    xlabel(ax4,"time [s]","Interpreter","latex");
    ylabel(ax4,"distance [m]","Interpreter","latex");
    xlim(ax4,[tMinUse,tMaxUse]);
    ylim(ax4,[0,1]);
    %yline(ax4,0.15,"--");

    aCell = {ax1,ax2,ax3,ax4};
end

function plotStacked(ax,t,name,styleCell)
    t = t(:,["time",name]);
    t = rmmissing(t);
    ax.NextPlot = "add";
    for iName = name
        plot(ax,t,"time",iName,styleCell{:});
    end
end