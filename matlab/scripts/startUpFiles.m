% check system dependency
if ~isunix
    error("Dependency LCQPow does not support non-linux system.")
end

% check DQ robotics dependency


% check submodules status
if exist("matlab/submodule/LCQPow/CMakeLists.txt","file") ~= 2
    [status,cmdout] = system("git submodule update --init --recursive");
    if status~=0
        error("Cloning submodule failed.\n%s",cmdout);
    else
        disp(cmdout)
    end
end

% check LCQPow build
if exist("matlab/submodule/LCQPow/build/lib/LCQPow","file") ~= 2 && exist("matlab/submodule/LCQPow/build/lib/LCQPow","file") ~= 3
    [status,cmdout] = system("cmake -B ./matlab/submodule/LCQPow/build -S ./matlab/submodule/LCQPow");
    if status~=0
        error("cmake failed.\n%s",cmdout)
    else
        disp(cmdout)
    end

    [status,cmdout] = system("make -C ./matlab/submodule/LCQPow/build");
    if status~=0
        error("make failed.\n%s",cmdout)
    else
        disp(cmdout)
    end
end

% check mcppath and pathmcp file existence
if exist("matlab/submodule/pathmex/pathmcp","file") ~= 2
    [status,cmdout] = system("wget -P ./matlab/submodule/pathmex https://pages.cs.wisc.edu/~ferris/path/matlab/pathmcp.m");
    if status~=0
        error("wget pathmcp failed.\n%s",cmdout)
    else
        disp(cmdout)
    end
end

if exist("matlab/submodule/pathmex/mcppath","file") ~= 3
    [status,cmdout] = system("wget -P ./matlab/submodule/pathmex https://pages.cs.wisc.edu/~ferris/path/matlab/mcppath.mexa64");
    if status~=0
        error("wget mcppath failed.\n%s",cmdout)
    else
        disp(cmdout)
    end
end

% codegen for mcpfuncjacEval (jacobian function provided to pathmcp)
if exist("matlab/codegenFile/mcpfuncjacEval2_mex","file") ~= 3
    codegenRun_pathmex;
end