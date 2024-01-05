function [f, J, domerr]= mcpfuncjacEval2(z,jacflag)
    %#codegen
    % MCPFUNCJACEVAL the jacobian function used in pathmex mcppath. The
    % function will be used as the 4th parameter (cpfj) of mcppath
    % Use mutiple global variables.
    global contact_normal; global dista; global q_o; global v_ts2js; %#ok<*GVMIS>
    global jcon_array;     global dof;   global h;   global safe_dist;
    global numColliLink;

    f = zeros(dof+numColliLink, 1); J=zeros(dof+numColliLink, dof+numColliLink); domerr = 0;
    invcon_array = zeros(dof, 6, numColliLink); q_n = z(1:dof);
      
    add_element = zeros(7, 1); temp_new = zeros(numColliLink, 1);
    first_part_mat = zeros(7, numColliLink); second_part_mat = zeros(numColliLink, 7);
    for num = 1:4
        temp_jac = jcon_array(:, :, num);
        invcon_array(1:num, 1:3, num) = pinv(temp_jac(1:3, 1:num));
        add_element = add_element + invcon_array(:, :, num)*contact_normal(:, num)*z(dof+num);
        temp_new(num, 1) = contact_normal(:, num)'*jcon_array(:, :, num)*(q_n-q_o);
        first_part_mat(:, num) = invcon_array(:, :, num)*contact_normal(:, num);
        second_part_mat(num, :) = contact_normal(:, num)'*jcon_array(:, :, num);
    end
    f(1:dof) = (q_o - q_n) + v_ts2js + h*add_element;
    f(dof+1 : end) = temp_new + (dista' - safe_dist);
        
    %if (jacflag)
        for i = 1:dof
            J(i, i) = -1;
        end
        J(1:dof, dof+1:dof+numColliLink) = h*first_part_mat;
        J(dof+1:dof+numColliLink, 1:dof) = second_part_mat;
        J=sparse(J);
    %else
end