clear
clc

outputPath = [pwd,'/matlab/codegenFile/'];

dof = 7;
h = 1e-3;
safe_dist = 1e-1;
jcon_array = zeros(6,7,4);
contact_normal = zeros(6,4);
dista = zeros(1,4);
q_o = zeros(7,1);
v_ts2js = zeros(7,1);
numColliLink = 4;

globalCell = {'dof',dof,'h',h,'safe_dist',safe_dist, ...
              'jcon_array',jcon_array,'contact_normal',contact_normal,'dista',dista, ...
              'q_o',q_o,'v_ts2js',v_ts2js,'numColliLink',numColliLink};

varargCell = {'-launchreport'};
codegenRun('mcpfuncjacEval2',{{zeros(11,1),1}},varargCell, ...
                outputPath=outputPath, ...
                outputName='mcpfuncjacEval2_mex', ...
                outputComment=false,...
                globalCell=globalCell);