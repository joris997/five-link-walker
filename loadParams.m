function p = loadParams()
% param_list = {'g','p(1)';
%               'L_torso','p(2)'; 
%               'L_fem','p(3)'; 
%               'L_tib','p(4)';
%               
%               'M_torso','p(5)'; 
%               'M_fem','p(6)'; 
%               'M_tib','p(7)';
%               
%               'MZ_torso','p(9)'; 
%               'MZ_fem','p(10)'; 
%               'MZ_tib','p(11)';
%               
%               'XX_torso','p(12)'; 
%               'XX_fem','p(13)'; 
%               'XX_tib','p(14)'};

% %% Original values from paper
%     p = [9.81;  % g
% 
%          0.625; % Ltorso
%          0.4;   % Lfemur
%          0.4;   % Ltibia
% 
%          12;    % Mtorso
%          6.8;   % Mfemur
%          3.2;   % Mtibia
% 
%          1.33;  % Inertia
%          0.47;
%          0.20
%          
%          0.24;  
%          0.11;
%          0.24];

%% From OpenSim model
    p = [9.81;  % g

         0.460; % Ltorso
         0.427; % Lfemur
         0.427; % Ltibia

         34.16; % Mtorso
         8.23;  % Mfemur
         4.57;  % Mtibia

         0.1032;  % Inertia
         0.1032;
         0.0382
         
         0.30;  
         0.12;
         0.24];
     
% %% From OpenSim model
%     p = [9.81;  % g
% 
%          0.460; % Ltorso
%          0.500; % Lfemur
%          0.500; % Ltibia
% 
%          34.16; % Mtorso
%          8.23;  % Mfemur
%          4.57;  % Mtibia
% 
%          0.1032;  % Inertia
%          0.1032;
%          0.0382
%          
%          0.30;  
%          0.12;
%          0.24];
end

