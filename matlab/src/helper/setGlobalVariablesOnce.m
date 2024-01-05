function setGlobalVariablesOnce(DOF,step,safeDistance)
    global dof;   global h;   global safe_dist; %#ok<*GVMIS>

    dof = DOF;
    h = step;
    safe_dist = safeDistance;
end

