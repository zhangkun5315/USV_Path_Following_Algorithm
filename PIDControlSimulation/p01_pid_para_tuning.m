clear all;
close all;
clc;
    
while true
    clear all;
    close all;
    clc;
    
    load_system('PID_basic');
    % pid_list = zeros(3, 9);
    
    files_list = dir('D:\_Work\9_Things\2_Match-USV_simulation\Simulink');
    
    for i=1:1:length(files_list)
        if strcmp(files_list(i).name, 'pid.txt')
            fid = fopen('D:\_Work\9_Things\2_Match-USV_simulation\Simulink\pid.txt', 'r');
            pid_para = fread(fid);
            fclose(fid);
            
            delete('D:\_Work\9_Things\2_Match-USV_simulation\Simulink\pid.txt')  % delete the para. file.
           % for j=1:1:9
            %    pid_list(1, j) = char(pid_para(j));
           % end
           % for j=11:1:19  % Index 10 is "space", from 11 start.
           %     pid_list(2, j-10) = char(pid_para(j));  
           % end
           % for j=21:1:29
            %    pid_list(3, j-20) = char(pid_para(j));
           % end
            
            pstr = strcat(char(pid_para(1)),char(pid_para(2)),char(pid_para(3)),char(pid_para(4)),char(pid_para(5)),char(pid_para(6)),char(pid_para(7)),char(pid_para(8)),char(pid_para(9)));
            
            istr = strcat(char(pid_para(11)),char(pid_para(12)),char(pid_para(13)),char(pid_para(14)),char(pid_para(15)),char(pid_para(16)),char(pid_para(17)),char(pid_para(18)),char(pid_para(19)));
            
            dstr = strcat(char(pid_para(21)),char(pid_para(22)),char(pid_para(23)),char(pid_para(24)),char(pid_para(25)),char(pid_para(26)),char(pid_para(27)),char(pid_para(28)),char(pid_para(29)));
                       
            set_param('PID_basic/P','Value',pstr);
            set_param('PID_basic/I','Value',istr);
            set_param('PID_basic/D','Value',dstr);
            sim('PID_basic',[0 100]);
            
            % After running, A new variable will be create in Workspace.
            fitness = simout(end);
            
            
            fid = fopen('D:\_Work\9_Things\2_Match-USV_simulation\Simulink\fitness.txt','w+');
            fprintf(fid,'%d',fitness); %
            fclose(fid);
            
            
        end
    end
    
    pause(0.1);
    
    
end




