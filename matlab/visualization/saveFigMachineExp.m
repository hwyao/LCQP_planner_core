tbl1 = readtable("./matlab/data/static_example_v2.csv","VariableNamingRule","preserve");
[f1,t1,aCell1] = plotConfiguration(tbl1,14,[2,8,14],0,0);
title(t1,"Static Scene");
ylim(aCell1{4},[0,0.6]);

tbl2 = readtable("./matlab/data/dynamic_example_v2.csv","VariableNamingRule","preserve");
[f2,t2,aCell2] = plotConfiguration(tbl2,11,[2,3,11],0,0);
title(t2,"Dynamic Scene");

print(f1,'-vector','-dsvg','./matlab/fig/static-data');
print(f2,'-vector','-dsvg','./matlab/fig/dynamic-data');