function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)

forceCmd.Fx_total   = 0;
forceCmd.brakeRatio = 0;

if vx > 24
    forceCmd.Fx_total   = -9000;
    forceCmd.brakeRatio = 0.55;
elseif vx > 20
    forceCmd.Fx_total   = -6500;
    forceCmd.brakeRatio = 0.40;
else
    forceCmd.Fx_total   = 0;
    forceCmd.brakeRatio = 0;
end

end