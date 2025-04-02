function save_plot(figure, file_name, folder_name)
% Simple function for saving plots as .fig and .png file.
% Note that a folder is created (with the name "folder_name") 
% in the same directory as the current path.
%
% INPUT:
% figure            : Handle to figure
% plot_name         : Name of .fig and .png file
% file_name         : Name of folder
% 
% OUTPUT: Generates two new plot files (.fig and .png) in specified
% folder from the figure handle in the input.
%

    % Create folder if not existing
    if (isfolder(folder_name) == false)
       mkdir(folder_name);
    end
    
    % Save plots
    saveas(figure, strcat(folder_name, '\', file_name, '.fig'));
    saveas(figure, strcat(folder_name, '\', file_name, '.png'));

end