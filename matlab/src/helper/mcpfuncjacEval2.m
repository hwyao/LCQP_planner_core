function [f, J, domerr]= mcpfuncjacEval2(z,jacflag)
    global contact_normal; global dista; global q_o; global v_ts2js;
    global jcon_array;     global dof;   global h;   global safe_dist;

    f = zeros(dof+4, 1); J=zeros(dof+4, dof+4); domerr = 0;
    invcon_array = zeros(dof, 6, 4); q_n = z(1:dof);
      
    add_element = zeros(7, 1); temp_new = zeros(4, 1);
    first_part_mat = zeros(7, 4); second_part_mat = zeros(4, 7);
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
    
    
    if (jacflag)
        for i = 1:dof
            J(i, i) = -1;
        end
        J(1:dof, dof+1:dof+4) = h*first_part_mat;
        J(dof+1:dof+4, 1:dof) = second_part_mat;
        J=sparse(J);
    end
end