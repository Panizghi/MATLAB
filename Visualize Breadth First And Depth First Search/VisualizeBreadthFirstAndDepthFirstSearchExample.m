%% Visualize Breadth-First and Depth-First Search
% This example shows how to define a function that visualizes the results
% of |bfsearch| and |dfsearch| by highlighting the nodes and edges of a
% graph.
%
% Create and plot a directed graph.

% Copyright 2017 The MathWorks, Inc.
s = [1 2 3 3 3 3 4 5 6 7 8 9 9 9 10];
t = [7 6 1 5 6 8 2 4 4 3 7 1 6 8 2];
G = digraph(s,t);
plot(G)

%%
% Perform a depth-first search on the graph. Specify |'allevents'| to
% return all events in the algorithm. Also, specify |Restart| as |true| to
% ensure that the search visits every node in the graph.
T = dfsearch(G, 1, 'allevents', 'Restart', true)

%%
% The values in the table, |T|, are useful for visualizing the search. The
% function |visualize_search.m| shows one way to use the results of
% searches performed with |bfsearch| and |dfsearch| to highlight the nodes
% and edges in the graph according to the table of events, |T|. The
% function pauses before each step in the algorithm, so you can slowly step
% through the search by pressing any key.
%
% Save |visualize_search.m| in the current folder.
%
% <include>visualize_search.m</include>
%

%%
% Use this command to run |visualize_search.m| on graph |G| and search
% result |T|:
%
%    visualize_search(G,T)
%

%%
% The graph begins as all gray, and then a new piece of the search result
% appears each time you press a key. The search results are highlighted
% according to:
%
% * |'startnode'| - Starting nodes _increase_ in size.
% * |'discovernode'| - Nodes turn _red_ as they are discovered.
% * |'finishnode'| - Nodes turn _black_ after they are finished.
% * |'edgetonew'| - Edges that lead to undiscovered nodes turn _blue_.
% * |'edgetodiscovered'| - Edges that lead to discovered nodes turn _magenta_.
% * |'edgetofinished'| - Edges that lead to finished nodes turn _green_.
%

%%
% This |.gif| animation shows what you see when you step through the
% results of |visualize_search.m|.
%
% <<../visualize_graph_search.gif>>
%