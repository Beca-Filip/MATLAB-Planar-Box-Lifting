clear all; close all; clc;

%% Process directory

% Get all files in directory
files = dir;

% Prefixes with which the extrema are stored
minpref = 'Storage_Minimum';
maxpref = 'Storage_Maximum';

% Cost function names
cfNames = containers.Map;

% Go through files and look for extrema
for ii = 1 : length(files)
    
    % Get current filename
    filename = files(ii).name;
    
    % For the files that match the minima storage pattern
    if startsWith(filename, minpref)
        
        % Load that file
        load(filename);
        
        % Remove prefix from name (replace it with empy char array)
        filename = strrep(filename, minpref, '');
        
        % Remove everything after first '_'
        if contains(filename, '_')
            id = strfind(filename, '_');
            filename = strrep(filename, filename(id(1):end), '');
        end
        
        % Use remaining string as cost function name
        cfname = filename;
        
        % Add to dictionary of cost functions if its not already in there
        % and set minimum value
        if ~cfNames.isKey(cfname)
            cfNames(cfname) = struct("Min", Storage.Results.f_star, "Max", []);
            
        % If it's already in dictionary, set only the minimum value
        else
            helper = cfNames(cfname);
            helper.Min = Storage.Results.f_star;
            cfNames(cfname) = helper;
        end
        
    end
    
    % For the files that match the maxima storage pattern
    if startsWith(filename, maxpref)
        
        % Load that file
        load(filename);
        
        % Remove prefix from name (replace it with empy char array)
        filename = strrep(filename, maxpref, '');
        
        % Remove everything after first '_'
        if contains(filename, '_')
            id = strfind(filename, '_');
            filename = strrep(filename, filename(id(1):end), '');
        end
        
        % Use remaining string as cost function name
        cfname = filename;
        
        % Add to dictionary of cost functions if its not already in there
        % and set minimum value
        if ~cfNames.isKey(cfname)
            cfNames(cfname) = struct("Min", [], "Max", -Storage.Results.f_star);
            
        % If it's already in dictionary, set only the maximum value
        else
            helper = cfNames(cfname);
            helper.Max = -Storage.Results.f_star;
            cfNames(cfname) = helper;
        end
    end
end

%% Plot a bar chart for each function

% Subplots in width
spw = 5;

% Retrieved names
names = cfNames.keys;

figure;
% For all cost functions
for ii = 1 : length(cfNames)
    
    % Create a subplot
    subplot(ceil(length(cfNames) / spw), spw, ii)
    
    % Plot a bars that represent the cost implication
    barValues = [cfNames(names{ii}).Min, cfNames(names{ii}).Max];
    numbars = size(barValues, 2);
    barLocations = 1:numbars;
    barNames = {'Min', 'Max'};
    hold on;
    barChart = bar(barLocations, barValues);
    xticks(barLocations);
    xticklabels(barNames);
    xtickangle(0);
    ylabel(names{ii});
end