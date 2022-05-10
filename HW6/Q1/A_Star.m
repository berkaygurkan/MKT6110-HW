%   MKT6110 Autonomous Robots HW6 - A* Algorithm      %
%   Author : Berkay GURKAN                            %
%   A* pathfinding algorithm shown below is used      %
%   https://www.youtube.com/watch?v=eSOJ3ARN5FM       %

%% A* Algoritm
clear
% Read Nodes and Edges
nodes = readmatrix('nodes.csv', 'CommentStyle','#')
edges = readmatrix('edges.csv','CommentStyle','#')
obstacles = readmatrix('obstacles.csv','CommentStyle','#')

% Specifiy Starting and Ending Nodes
STARTING_NODE = 1;
END_NODE = 12;

%% Initialize variables
f = Inf;
current_node = STARTING_NODE;
end_node = END_NODE;
OpenList = [STARTING_NODE,f];
ShowList = [STARTING_NODE,f,0,0];
ClosedList =0



while current_node ~= end_node
    
    % Find adjacent node row index(es) 
    RowIndex = find(edges(:,2)==current_node);
    k = length(RowIndex);

    % For each adjacent node to current node
    for i=1:1:k
        if  isempty((find(ClosedList == edges(RowIndex(i))))) && ...       % If adjacent node is not in the closed list and open list
            isempty((find(OpenList(:,1) == edges(RowIndex(i)))))
            
            exec_node = edges(RowIndex(i),1)
            disp( ['Adjacent Node ',num2str(exec_node),' is not in closed list or opened list'])

            % Calculate distance from start (g) [Euclidian distance]
            if ~isempty(find(ShowList == current_node))                    % Find previous distance using table
                prev_index = find(ShowList(:,1) == current_node);
                prev_distance = ShowList(prev_index,3);
            else
                prev_distance = 0;
            end
            
            g = edges(RowIndex(i),3)+ prev_distance;                       % Calcualate previous distance 
            h = nodes(exec_node,4);                                        % Fetch heuristic value from array
            f_new = g + h;                                                 % Calculate total distance
            parent = current_node;                                         % Specify parent for adjacent node
            
            % Add new node to open list
            OpenList(end+1,:) = [exec_node,f_new];
            
            % Show List for all table and previous distance information
            ShowList(end+1,:) = [exec_node,f_new,g,parent];
            
            % Update F value
            if f_new < f        
                f = f_new;
            end
        end
        
        
    end
    
      % Add executed node to closed list
        ClosedList(end+1) = current_node;
        
        % Remove smallest f node from openlist and make it current
        [H,I]=min(OpenList(:,2));
        current_node=OpenList(I,1);
        OpenList(I,1:2) = NaN;
        disp(['Current Node is ',num2str(current_node)]);

        % Display Closed and Open List for debug purposes
        %disp(ClosedList)
        %disp(OpenList)
       
    
end


%% Calculate Path and Save on .csv
FlippedFinalPath = zeros(0,0)
next_index = END_NODE

while next_index ~= STARTING_NODE

    index = find(ShowList(:,1) == next_index);
    FlippedFinalPath(end+1) = ShowList(index,1);
    next_index = ShowList(index,4);

end

disp("Path is found by using A* algortihm")
FinalPath = flip(FlippedFinalPath);
disp(FinalPath)


writematrix(FinalPath,'myPath.csv')