function setGlobalVariables(contactJacobMtx, contactNormalMtx, contactDistMtx, ip_task2js, qNow, nColliLink)
    global jcon_array; global contact_normal; global dista; %#ok<*GVMIS>
    global q_o; global v_ts2js; global numColliLink; 

    % set the gloabal variables
    jcon_array = contactJacobMtx;
    contact_normal = contactNormalMtx;
    dista = contactDistMtx;
    v_ts2js = ip_task2js;
    q_o = qNow;
    numColliLink = nColliLink;
end