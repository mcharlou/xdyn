function H = xdyn_plotPositions(states)
% XDYN_PLOTPOSITIONS creates a figure that displays the evolution of
% positions and rotations.
%
% Input:
%   - states : Structure containing time and positions information
%
% Output:
%   - H : Handle to the created figure
%
% Example :
%
%    simu = xdyn_loadResultsFromHdf5File('simu.hdf5');
%    xdyn_plotPositions(simu.states);
%
% See also xdyn_loadResultsFromHdf5File
H = figure;
yLabel = {'X (m)','Y (m)','Z (m)','Phi (deg)','Theta (deg)','Psi (deg)'};
scale = [1 1 1 180/pi 180/pi 180/pi];
bodyColor = '-bgmyk';
names = fieldnames(states);
nObject = numel(fieldnames(states));
XYZEul = cell(1,nObject);
for i=1:nObject
    n = names{i};
    XYZEul{i} = [states.(n).x, states.(n).y, states.(n).z, states.(n).eul];
end
if nObject==1
    bodyColor = 'k';
end
if isempty(states.(n).eul)
    nRow = 1;
    nCol = 3;
else
    nRow = 2;
    nCol = 6;
end
for j=1:nCol
    subplot(nRow,3,j);
    box on
    grid on
    hold on
    xlabel('T (s)');
    ylabel(yLabel{j});
    for i=1:nObject
        n = names{i};
        plot(states.(n).t, scale(j)*XYZEul{i}(:,j), bodyColor(i));
    end
end
return