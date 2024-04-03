function tblOut = runTask(taskClass,positionList,radiusList,config,goal)
    taskClass.init(positionList,radiusList,config,goal);
    taskClass.run();
    tblOut = taskClass.dataTable;
end

