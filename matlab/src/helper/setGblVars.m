function [] = setGblVars(contactJacobMtx, contactNormalMtx, contactDistMtx, ip_task2js, qNow)
    global jcon_array; global contact_normal; global dista;
    global q_o; global v_ts2js;

    % set the gloabal variables
    jcon_array = contactJacobMtx;
    contact_normal = contactNormalMtx;
    dista = contactDistMtx;
    v_ts2js = ip_task2js;
    q_o = qNow;
    
end