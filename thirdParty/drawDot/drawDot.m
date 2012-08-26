% This code loads a DOT file without coordinates given, passes it to
% GraphViz, and draws a graph in the axis specified.
% Most graphs are currently supported, but support is not
% possible in some cases (e.g. record nodes) because the plain output
% format required does not provide the information necessary.
% This interface written by Bjorn Wielens - Copyright 2009. 
% Please leave this copyright notice intact, and if possible, 
% provide credit for use of this code.
%
% KNOWN ERRATA AND NOTES:
% -mwdot spline points always go from left to right, regardless of arrow
% direction. This is a bug in the mwdot engine.
% -The 'polygon' shape is drawn as a heptagon, since plain output (-Tplain)
% offers no polygon coordinates.
% -Only the X11 colour scheme is supported for colour names. These are
% defined in the routine defineX11Colours()
% -Plain output offers no arrow style information.
% -'diagonals' node style is not supported, but the three shapes
% 'msquare','mdiamond', and 'mcircle' are.
% -Plain output does not specify background colours. These must be set
% manually for the axes.
% - Only 'single' arrows are supported - shapes such as 'normalnormal' are
% not.
% - To have MATLAB display the graph with proper aspect ratios, set the
%'DataAspectRatio' property of your axes to [1 1 1].
% - Shapes that normally have double/triple outlines do not actually show
% these outlines. For simplicity's sake, the linewidth of the edge of the
% shape is increased. If you need this feature, feel free to contact me- I
% do have experimental code that could be used.
%
%
% Format:
% [textObj,patchObj,nodeNames,adjacent]=drawDot(dotFile,surface,parameters)
%
% INPUT ARGUMENTS:
% dotFile is the name of the dot file (including path)
% 
% surface is the handle of the axis. The axis is cleared before
% drawing.
%
% The following parameters are available to be passed as parameter/value pairs:
% colourMode,arrowStyle,layoutEngine,autoSize,edgeResolution,fullMode
%
% colourMode specifies whether colour triplets are RGB or hsv triplets, 
% by setting it to either 'hsv' or 'rgb'. Default assumes RGB. Ex:
% 'colourMode','rgb' in the arguments.
%
% arrowStyle lets you specify the arrow style to include on the graph,
% since -Tplain output does not give arrow information. If not specified,
% the default is 'normal'. For no arrows, specify 'none'. You can
% additionally use '<'/'>' to specify reverse/forward arrows. specifying both
% gives two-directional arrows. For example: '<normal','>normal','<>normal'
%
% layoutEngine lets you specify a custom layout engine, e.g. twopi, neato,
% etc. If none is given, the default is 'dot', followed by 'mwdot', only if
% the former is not found. If the engine is not in the path, you must also
% give the full path in DOS "8.3" format, e.g.
% 'c:\progra~1\Graphviz2.22\bin\neato.exe'
% 
% autoSize, when true, automatically sets the figure's window to the
% actual size of the graph (or the full screen,whichever is smaller)
% , as defined in the DOT file. When false, you must manually set the
% window size.
%
% edgeResolution specifies the number of points to use for each B-spline
% edge. The calculations can be intensive, so for complex graphs, a lower
% value such as 25 (default=100) is recommended. Higher values increase the
% smoothness of the line, but also the time needed to calculate the lines.
% The bare minimum resolution is 10. Anything less than this is ignored.
%
% fullMode (true/false) enables/disables all features of the plot routines. (Enabled by
% default). Disabling it offers little performance gain. The resulting
% graph has no patches for node shapes, using instead the border of the
% text object to draw a box around each piece of text. This makes
% "interactive" graphs easier to implement, since the box will always fit around the
% text, even if it is changed after the original plot has completed.
% Zooming and text scaling features are also disabled when fullMode is set
% to false.
%
% OUTPUT:
% textObj returns the handles to the _node_ text objects in the graph.
% patchObj returns the handles to each node's patch, in the same order.
% nodeNames returns the names of each node, in the same order as the above.
% adjacent returns an adjacency matrix for the graph. Finding a '1' in
% row M, column N of the adjacency matrix means that M is a parent of N, or
% conversely, that N is a child of M.

function [textObj,patchObj,nodeNames,adjacent]=drawDot(dotFile,surface,varargin)
arrow('clear');
if numel(varargin)==1 && iscell(varargin{1})
    varargin=varargin{1};
end
if mod(numel(varargin),2)~=0
    error('drawDot:MissingArguments','You do not have a complete set of parameter/value arguments');
end
autoSize=false;
colourMode='rgb';
arrowStyle='>normal';
edgeResolution=100;
fullMode=true;
for i=1:2:numel(varargin)
    switch lower(varargin{i})
        case 'colourmode'
            colourMode=varargin{i+1};
        case 'layoutengine'
            layoutEngine=varargin{i+1};
        case 'arrowstyle'
            arrowStyle=varargin{i+1};
        case 'autosize'
            autoSize=varargin{i+1};
        case 'edgeresolution'
            edgeResolution=varargin{i+1};
            if edgeResolution <10 
                edgeResolution=10;
            end
        case 'fullmode'
            fullMode=varargin{i+1};
    end
end
if exist('layoutEngine','var')
    engine=layoutEngine;
    if ~isempty(regexp(layoutEngine,'mw(dot|twopi)', 'once'))
        warning('drawDot:BuggyEngine','You appear to be using MATLAB''s mwtwopi or mwdot engine. These engines may have bugs that cause unexpected behaviour.');
    end 
else
    engine='dot';
    matlabEngine='mwdot';
end
[status,returnCode]=system([engine ' -Tplain ' dotFile]); 
if status~=0 && exist('matlabEngine','var')
    warning('drawDot:InstallGraphViz',['Graphviz was not found installed on your system (not in path?). Reverting to MATLAB version...' ...
        ' NOTE: The matlab version of mwdot contains bugs which may cause erratic behaviour. Using graphviz is recommended.' ... 
        ' If you have just installed it, try restarting MATLAB.']);
    [status,returnCode]=system([matlabEngine ' -Tplain ' dotFile]); 
end
if status~=0
    error('drawDot:EngineError',['An error occurred when trying to run the layout engine. The system said, "' returnCode '".']);
end
cla(surface);
figureHandle=get(surface,'Parent');
textObj=[];
patchObj=[];
nodeNames={};
fileContent=regexp(returnCode,'\n','split')';
graphLine=fileContent{~cellfun('isempty',regexp(fileContent,'^graph'))};
tokens=regexp(graphLine,'\s+','split');
axes(surface);
hold on;
scale=str2double(tokens{2});
hOffset=str2double(tokens{3})*scale*0.05;
vOffset=str2double(tokens{4})*scale*0.05;
axis([0-hOffset (str2double(tokens{3})*scale)+vOffset 0-vOffset (str2double(tokens{4})*scale)+vOffset]);
set(surface,'xTick',[],'YTick',[]);
if autoSize
    set(figureHandle,'units','inches','position',[0 0 str2double(tokens{3})*scale str2double(tokens{4})*scale]);
    set(figureHandle,'units','pixels');
    centerfig(figureHandle);
    figSize=get(figureHandle,'Position');
    screenSize=get(0,'ScreenSize');
    if ispc
        taskbarHeight=34;
    elseif isunix
        taskbarHeight = 50;
    end
    figSize(2)=max(taskbarHeight,figSize(2));
    figSize(1)=max(figSize(1),screenSize(1));
    figSize(3)=min(screenSize(3),figSize(3));
    figSize(4)=min(screenSize(4)-taskbarHeight-6,figSize(4)-taskbarHeight-6);
    set(figureHandle,'Position',figSize);
end
nodeLines=fileContent(~cellfun('isempty',regexp(fileContent,'^node')));
tokens=regexp(nodeLines,'node\s+(?<nodeID>"[^"]+"|[\w\d]+)\s+(?<x>[\d\.\+e]+)\s+(?<y>[\d\.\+e]+)\s+(?<w>[\d\.\+e]+)\s+(?<h>[\d\.\+e]+)\s+(?<label>".*"|[\w\d]+)\s+(?<style>\w+)\s+(?<shape>[\w\d]+)\s+(?<edgeColour>(?:[#\w][\w\d]+)|[\d\.]+\s+[\d\.]+\s+[\d\.]+)\s+(?<bgColour>(?:[#\w][\w\d]+)|[\d\.]+\s+[\d\.]+\s+[\d\.]+)','tokens');
tokens=[tokens{:}]';
tokens=[tokens{:}]';
tokens=reshape(tokens,10,[])';
nodeIDs=regexprep(tokens(:,1),'"','');
nodeNames=nodeIDs;
x=cellfun(@str2double,tokens(:,2))*scale;
y=cellfun(@str2double,tokens(:,3))*scale;
w=cellfun(@str2double,tokens(:,4))*scale;
h=cellfun(@str2double,tokens(:,5))*scale;
labels=regexprep(tokens(:,6),'"','');
shapes=tokens(:,8);
edgeColours=cellfun(@(a)getColour(a,colourMode),tokens(:,9),'UniformOutput',false);
bgColours=cellfun(@(a)getColour(a,colourMode),tokens(:,10),'uniformOutput',false);
properties=regexprep(tokens(:,7),{'dashed','solid','dotted','bold','invis','filled','diagonals'},{'lineStyle','lineStyle','lineStyle','lineWidth','visible','faceColor','lineStyle'});
values=regexprep(tokens(:,7),{'dashed','solid','dotted','bold','invis','filled','diagonals'},{'--','-',':','bold','off','faceColour','diags'});
boldReplace=~cellfun('isempty',regexp(values,'bold'));
colourReplace=~cellfun('isempty',regexp(values,'faceColour'));
blackEdges=cellfun(@(a)all(~a),edgeColours);
diagReplace=~cellfun('isempty',regexp(values,'diags'));
if any(diagReplace)
    warning('drawDot:NotSupported','Diagonal styles are not supported. Defaulting to dash-dot line.');
    values(diagReplace)='-.';
end
blackEdges=and(colourReplace,blackEdges);
values(colourReplace)=edgeColours(colourReplace);
values(blackEdges)=bgColours(blackEdges);
values(boldReplace)={2};
if fullMode
    [patchObj]=cellfun(@drawNode,shapes,num2cell(x),num2cell(y),num2cell(w),num2cell(h),edgeColours,bgColours,properties,values,'UniformOutput',false);
    textObj=text(x,y,labels,'HorizontalAlignment','center','Interpreter','none','verticalAlignment','middle','margin',0.1);
    patchObj=cell2mat(patchObj);
else
    textObj=text(x,y,labels,'HorizontalAlignment','center','Interpreter','none','verticalAlignment','middle','margin',2,'edgeColor',[0 0 0]);
end
edgeLines=fileContent(~cellfun('isempty',regexp(fileContent,'^edge')));
tokens=regexp(edgeLines,'edge\s+(?<tail>(?:"[^"]+")|(?:[\w\d]+))\s+(?<tip>(?:"[^"]+")|([\w\d]+))\s+(?<numSpline>\d+)\s+([\d\.\+e\s]+)\s+((".*"|[\w\d]+)\s+([\d\.\+e]+)\s+([\d\.\+e]+)\s+)?(\w+)\s+(?<bgColour>(?:[#\w][#\w\d:]+)|[\d\.]+\s+[\d\.]+\s+[\d\.]+)','tokens');
tokens=[tokens{:}]';
tokens=[tokens{:}]';
tokens=reshape(tokens,7,[])';
tailNodes=regexprep(tokens(:,1),'"','');
[NULL,tailLocation]=ismember(tailNodes,nodeIDs);
tailCoord=[x(tailLocation)';y(tailLocation)'];
tipNodes=regexprep(tokens(:,2),'"','');
[NULL,tipLocation]=ismember(tipNodes,nodeIDs);
adjacent=sparse(numel(nodeIDs),numel(nodeIDs));
index=sub2ind(size(adjacent),tailLocation,tipLocation);
adjacent(index)=true;
tipCoord=[x(tipLocation)';y(tipLocation)'];
numSpline=num2cell(str2double(tokens(:,3)));
splinePts=cellfun(@str2num,tokens(:,4),'uniformoutput',false);
splinePts=cellfun(@(a,b)reshape(a*scale,[],b),splinePts,numSpline,'uniformoutput',false);
splinePts=cellfun(@horzcat,num2cell(tailCoord,1)',splinePts,num2cell(tipCoord,1)','Uniformoutput',false);
space=repmat(linspace(0,1,edgeResolution),numel(splinePts),1);
splinePts=cellfun(@transpose,splinePts,'uniformoutput',false);
[NULL,NULL,splinePts]=cellfun(@bezier,num2cell(zeros(numel(splinePts),1)),num2cell(ones(numel(splinePts),1)),splinePts,num2cell(space,2),'Uniformoutput',false);
splinePts=cell2mat(splinePts);
splineX=num2cell(reshape(splinePts(:,1),[],numel(numSpline))',2);
splineY=num2cell(reshape(splinePts(:,2),[],numel(numSpline))',2);
labels=tokens(:,5);
labels=labels(~cellfun('isempty',labels));
if ~isempty(labels)
    labels=regexp(labels,'("[\w\d\s]*"|[\w\d]+)\s+([\d\.]+)\s+([\d\.]+)','tokens');
    labels=[labels{:}]';
    labels=[labels{:}]';
    labels=reshape(labels,3,[])';
    labelText=regexprep(labels(:,1),'"','');
    empty=cellfun('isempty',labelText);
    if any(~empty)
        labelX(~empty)=cellfun(@str2double,labels(~empty,2))*scale;
        labelY(~empty)=cellfun(@str2double,labels(~empty,3))*scale;
        text(labelX(~empty),labelY(~empty),labelText(~empty),'HorizontalAlignment','center','verticalAlignment','middle','interpreter','none');
    end
end
colours=cellfun(@(a)getColour(a,colourMode),tokens(:,7),'UniformOutput',false);
properties=regexprep(tokens(:,6),{'dashed','solid','dotted','bold','invis'},{'lineStyle','lineStyle','lineStyle','lineWidth','lineStyle'});
values=regexprep(tokens(:,6),{'dashed','solid','dotted','bold','invis'},{'--','-',':','bold','none'});
values(~cellfun('isempty',regexp(values,'bold')))={2};
multiColour=cellfun(@iscell,colours);
if fullMode
    tipPolyX=get(patchObj(tipLocation),'XData');
    tipPolyY=get(patchObj(tipLocation),'YData');
    tailPolyX=get(patchObj(tailLocation),'XData');
    tailPolyY=get(patchObj(tailLocation),'YData');
else
    tipPoly=get(textObj(tipLocation),'Extent');
    tipPolyX=cellfun(@(a) [a(1) a(1) a(1)+a(3) a(1)+a(3)],tipPoly,'uniformoutput',false);
    tipPolyY=cellfun(@(a) [a(2)+a(4) a(2) a(2) a(2)+a(4)],tipPoly,'uniformoutput',false);
    tailPoly=get(textObj(tailLocation),'Extent');
    tailPolyX=cellfun(@(a) [a(1) a(1) a(1)+a(3) a(1)+a(3)],tailPoly,'uniformoutput',false);
    tailPolyY=cellfun(@(a) [a(2)+a(4) a(2) a(2) a(2)+a(4)],tailPoly,'uniformoutput',false);
end
inPolygon=cellfun(@inpolygon,splineX(~multiColour),splineY(~multiColour),tipPolyX(~multiColour),tipPolyY(~multiColour),'uniformoutput',false);
inPolygon2=cellfun(@inpolygon,splineX(~multiColour),splineY(~multiColour),tailPolyX(~multiColour),tailPolyY(~multiColour),'uniformoutput',false);
inPolygon=cellfun(@not,cellfun(@or,inPolygon,inPolygon2,'uniformoutput',false),'uniformoutput',false);
splineX(~multiColour)=cellfun(@(a,i) a(i),splineX(~multiColour),inPolygon,'uniformoutput',false);
splineY(~multiColour)=cellfun(@(a,i) a(i),splineY(~multiColour),inPolygon,'uniformoutput',false);
cellfun(@(a,b,c,d,e)plot(a,b,c,d,'Color',e),splineX(~multiColour),splineY(~multiColour),properties(~multiColour),values(~multiColour),colours(~multiColour));
if any(multiColour)
    [splineX(multiColour),splineY(multiColour)]=cellfun(@multi_Coords,splineX(multiColour),splineY(multiColour),colours(multiColour),tipPolyX(multiColour),tipPolyY(multiColour),tailPolyX(multiColour),tailPolyY(multiColour),'uniformoutput',false);
    cellfun(@multi_Plot,splineX(multiColour),splineY(multiColour),properties(multiColour),values(multiColour),colours(multiColour));
end
if ~isempty(regexp(arrowStyle,'>','once'))
    fwdArrow=true;
else
    fwdArrow=false;
end
if ~isempty(regexp(arrowStyle,'<','once'))
    revArrow=true;
else
    revArrow=false;
end
if fullMode
    temp.Axes=surface;
    set(textObj,'FontUnits','normalized');
    cellfun(@resizeText,num2cell(textObj),num2cell(patchObj));
    fontSizes=get(textObj,'fontSize');
    setappdata(figureHandle,'fontSizes',cell2mat(fontSizes));
    setappdata(surface,'originalX',get(surface,'Xlim'));
    setappdata(surface,'originalY',get(surface,'Ylim'));
    setappdata(figureHandle,'patches',patchObj);
    setappdata(figureHandle,'texts',textObj);
end
arrowStyle=regexprep(arrowStyle,'[\<\>]','');
noLines=or(cellfun('length',splineX)<2,cellfun('length',splineY)<2);
splineX(multiColour)=cellfun(@getPoints,splineX(multiColour),'uniformoutput',false);
splineY(multiColour)=cellfun(@getPoints,splineY(multiColour),'uniformoutput',false);
colours(multiColour)=cellfun(@cell2mat,colours(multiColour),'uniformoutput',false);
if revArrow
    arrowX=cellfun(@(a) a(end,[1 2]),splineX(~noLines),'uniformoutput',false);
    arrowY=cellfun(@(a) a(end,[1 2]),splineY(~noLines),'uniformoutput',false);
    cellfun(@(a,b,c) arrow(arrowStyle,a(1,:),b,c),colours(~noLines),arrowX,arrowY);
end
if fwdArrow
    arrowX=cellfun(@(a) a(1,[end end-1]),splineX(~noLines),'uniformoutput',false);
    arrowY=cellfun(@(a) a(1,[end end-1]),splineY(~noLines),'uniformoutput',false);
    cellfun(@(a,b,c) arrow(arrowStyle,a(end,:),b,c),colours(~noLines),arrowX,arrowY);
end
if fullMode
    zoomHandle=zoom(figureHandle);
    rotateHandle=rotate3d;
    setAllowAxesRotate(rotateHandle,surface,false);
    set(zoomHandle,'ActionPostCallback',@zoom_callback);
end


function points=getPoints(splineArray)
points(1,1:2)=splineArray{end}(end-1:end);
points(2,1:2)=splineArray{1}(1:2);


function [splineX,splineY]=multi_Coords(splineX,splineY,colours,tipPolyX,tipPolyY,tailPolyX,tailPolyY)
numColours=numel(colours);
rangeTipX=(max(tipPolyX)-min(tipPolyX));
rangeTipY=(max(tipPolyY)-min(tipPolyY));
rangeTailX=(max(tailPolyX)-min(tailPolyX));
rangeTailY=(max(tailPolyY)-min(tailPolyY));
if rangeTailX<rangeTipX
    xOffsets=linspace(min(tailPolyX),max(tailPolyX),numColours+6)';
    xOffsets=xOffsets-min(tailPolyX)-(rangeTailX/2);
else
    xOffsets=linspace(min(tipPolyX),max(tipPolyX),numColours+6)';
    xOffsets=xOffsets-min(tipPolyX)-(rangeTipX/2);
end
if rangeTailY<rangeTipY
    yOffsets=linspace(min(tailPolyY),max(tailPolyY),numColours+6)';
    yOffsets=yOffsets-min(tailPolyY)-(rangeTailY/2);
else
    yOffsets=linspace(min(tipPolyY),max(tipPolyY),numColours+6)';
    yOffsets=yOffsets-min(tipPolyY)-(rangeTipY/2);
end
xOffsets=repmat(xOffsets(4:end-3),1,numel(splineX));
yOffsets=repmat(yOffsets(4:end-3),1,numel(splineY));
splineX=repmat(splineX,numColours,1);
splineY=repmat(splineY,numColours,1);

if abs(asind((max(splineY(1,:))-min(splineY(1,:)))/(max(splineX(1,:))-min(splineX(1,:)))))<40
    splineY=splineY+yOffsets;
end
if abs(asind((max(splineY(1,:))-min(splineY(1,:)))/(max(splineX(1,:))-min(splineX(1,:)))))>40
    splineX=splineX+xOffsets;
end
splineX=num2cell(splineX,2);
splineY=num2cell(splineY,2);
inPolygon=cellfun(@(a,b)inpolygon(a,b,tipPolyX,tipPolyY),splineX,splineY,'uniformoutput',false);
inPolygon2=cellfun(@(a,b)inpolygon(a,b,tailPolyX,tailPolyY),splineX,splineY,'uniformoutput',false);
inPolygon=cellfun(@not,cellfun(@or,inPolygon,inPolygon2,'uniformoutput',false),'uniformoutput',false);
splineX=cellfun(@(a,i)a(i),splineX,inPolygon,'uniformoutput',false);
splineY=cellfun(@(a,i)a(i),splineY,inPolygon,'uniformoutput',false);

function multi_Plot(splineX,splineY,property,value,colours)
cellfun(@(a,b,c)plot(a,b,property,value,'color',c),splineX,splineY,colours);

function zoom_callback(hObject,eventData)
oldX=getappdata(eventData.Axes,'originalX');
oldY=getappdata(eventData.Axes,'originalY');
newX=get(eventData.Axes,'Xlim');
newY=get(eventData.Axes,'Ylim');
dx=newX(2)-newX(1);
dy=newY(2)-newY(1);
if dx<(oldX(2)-oldX(1))
    % zoom in
    if dx<dy
        newScale=(oldX(2)-oldX(1))/dx;
    else
        newScale=(oldY(2)-oldY(1))/dy;
    end
else
    % zoom out
    if dx>dy
        newScale=(oldX(2)-oldX(1))/dx;
    else
        newScale=(oldY(2)-oldY(1))/dy;
    end
end
fontSizes=getappdata(hObject,'fontSizes');
textObjs=getappdata(hObject,'texts');
fontSizes=fontSizes*newScale;
fontSizes(fontSizes>100)=100;
fontSizes=num2cell(fontSizes);
cellfun(@(a,b)set(a,'fontSize',b),num2cell(textObjs),fontSizes);


function resizeText(thisText,thisPatch)
if isempty(get(thisText,'String'))
    return;
end
set(thisText,'FontSize',0.001);
fontSize=0.001;
patchX=get(thisPatch,'XData');
patchY=get(thisPatch,'YData');
textExtent=get(thisText,'extent');
textX=[textExtent(1),textExtent(1)+textExtent(3)];
textY=[textExtent(2),textExtent(2)+textExtent(4)];
textOK=inpolygon(textX,textY,patchX,patchY);
while all(textOK)
    fontSize=get(thisText,'FontSize');
    set(thisText,'FontSize',fontSize*1.25);
    textExtent=get(thisText,'extent');
    textX=[textExtent(1),textExtent(1)+textExtent(3)];
    textY=[textExtent(2),textExtent(2)+textExtent(4)];
    textOK=inpolygon(textX,textY,patchX,patchY);
end
set(thisText,'FontSize',fontSize);

%This function takes a colour either in hex or in the X11 scheme, and
%returns the matlab RGB code for it.
function thisColour=getColour(thisColour,colourMode)
persistent colours;
if isempty(colours)
    colours=defineX11Colours();
end
if any(thisColour==58)
    %multi-colour!
    newColours=regexp(thisColour,':','split');
    thisColour=cellfun(@(a)getColour(a,colourMode),newColours,'uniformoutput',false);
    thisColour=thisColour';
    return;
end
if strmatch('#',thisColour)
    %hex code
    thisColour=[hex2dec(thisColour(2:3)) hex2dec(thisColour(4:5)) hex2dec(thisColour(6:7))];
    thisColour=thisColour./255;
    return;
end
if any(thisColour==32) %spaces - must be triplet.
    thisColour=str2num(thisColour);
    if strcmp(colourMode,'hsv')
        thisColour=hsv2rgb(thisColour);
    end
    return;
end
try
    thisColour=colours.(thisColour);
catch noSuchColour
    warning('drawDot:ColourUndefined',['Colour "' thisColour '" not defined. Please use hex values, or edit the definitions manually to add it. Defaulting to white.']);
    thisColour=[1 1 1];
end

% Bezier function written by Jonas Ballani. 
% Inputs: a=left edge
% b=right edge, 
% c= control points
% y= series of linear points to use for bezier.
function [X,Y,val] = bezier(a,b,p,y)

% function val = bezier(a,b,p,y)
% bezier(0,1,,linspace(0,1,nplot))
% INPUT:  a   Linke Intervallgrenze
%         b   Rechte Intervallgrenze
%         p   St�tzstellen (nx2-Matrix)
%         y   Auswertungspunkte (Spaltenvektor)
%
% OUTPUT: val   Werte des Bezierpolynoms an y (mx2-Matrix)
%
% Date:   2007-11-05
% Author: Jonas Ballani

n = size(p,1);
m = length(y);
T = zeros(n,n);
val = zeros(m,2);
X(:,1) = p(:,1);
Y(:,1) = p(:,2);

for j = 1:m
    for i = 2:n
        X(i:n,i) = (b-y(j))/(b-a)*X(i-1:n-1,i-1) + (y(j)-a)/(b-a)*X(i:n,i-1);
        Y(i:n,i) = (b-y(j))/(b-a)*Y(i-1:n-1,i-1) + (y(j)-a)/(b-a)*Y(i:n,i-1);
    end
    val(j,1) = X(n,n);
    val(j,2) = Y(n,n);
end
 
% Simple function to draw an arrow head on the end of a line- tipX/Y are
% the coordinates of the arrow's tip, and tailX/Y are coordinates of where
% the arrow's tail points. (no tail is actually drawn.) Finally,
% the style argument is the type of pre-fab arrowhead to use. These can be
% defined in the switch statement as necessary. See
% http://graphviz.org/doc/info/attrs.html#k:style for info on the basic
% shapes supported/arrow grammar. 

function arrow(style,colour,x,y)
persistent scale
if strcmp(style,'clear')
    clear persistent scale;
    return;
end
tipX=x(1);tailX=x(2);tipY=y(1);tailY=y(2);
if isempty(scale) || scale==0
    scale=findobj('type','text','fontunits','normalized');
    if isempty(scale) 
        scale=text(0,0,'a');
        textHandle=scale(1);
        scale=get(scale(1),'extent');
        scale=scale(4)/1.5;
        delete(textHandle);
    else
        scale=get(scale(1),'extent');
        scale=scale(4);
    end
    if scale==0
        scale=text(1,1,'a');
        textHandle=scale;
        scale=get(scale,'Extent');
        scale=scale(4)/1.5;
        delete(textHandle);
    end
end
isInverted=false;
if strmatch('inv', style)
    isInverted=true;
    if strcmp(style,'inv')
        style='normal';
    else
        style=style(4:end);
    end
end
if (strcmp('o',style(1))||strcmp('e',style(1))) && ~strcmp('open','style')
    noFill=true;
    style=style(2:end);
else
    noFill=false;
end
if strcmp('l',style(1))
    side='left';
    style=style(2:end);
elseif strcmp('r',style(1))
    side='right';
    style=style(2:end);
else
    side='all';
end
length=sqrt((tailX-tipX)^2+(tailY-tipY)^2);
theta=asind((tipX-tailX)/length);
if tipY>tailY
    theta=-theta+180;
end
rotMatrix=[cosd(theta) -sind(theta); sind(theta) cosd(theta)];
switch style
    % add defines for other styles here... These are just the base patch
    % that will get transformed later. Shapes should fill the unit square
    % and center at zero. These are scaled/shifted later.
    case 'empty'
        patchX=[0,0.5,-0.5];
        patchY=[-0.5,.5,0.5];
        noFill=true;
    case 'normal'
        patchX=[0,0.5,-0.5];
        patchY=[-0.5,.5,0.5];
    case {'vee','open'}
        patchX=[0,0.5,0,-0.5];
        patchY=[-0.5,0.5,0.2,0.5];
    case 'halfopen'
        patchX=[0,0.5,0];
        patchY=[-0.5,0.5,0.2];
    case 'crow'
        patchX=[0,0.5,0,-0.5];
        patchY=[0.5,-0.5,-0.2,-0.5];
    case 'dot'
        patchX=-1:0.05:1;
        patchY=[sqrt(1-patchX.^2) -sqrt(1-patchX.^2)];
        patchX=[patchX fliplr(patchX)];
        patchX=patchX.*0.5; patchY=patchY.*0.5;
    case 'tee'
        patchX=[-0.5,0.5,0.5,-0.5];
        patchY=[-0.5,-0.5,-0.2,-0.2];
    case 'box'
        patchX=[-0.5,0.5,0.5,-0.5];
        patchY=[-0.5,-0.5,0.5,0.5];
    case 'diamond'
        patchX=[-0.5,0,0.5,-0];
        patchY=[0,0.5,0,-0.5];
    case 'none'
        return;
    otherwise
        warning('drawDot:UnsupportedArrow',['The arrow style ' style ' is not defined. You can define it manually in the arrow function.']);
        return;
end
if isInverted 
    patchY=-patchY;
end
if tipY>tailY
    patchX=-patchX;
end
switch side
    case 'left'
        patchX(patchX<0)=0;
    case 'right'
        patchX(patchX>0)=0;
    case 'all'
end
patchY=patchY+0.5;
patchX=patchX*scale;
patchY=patchY*scale;
drawOut=rotMatrix*[patchX; patchY];
if noFill
    patch((drawOut(1,:)+tipX),(drawOut(2,:)+tipY),'FaceColor',[1 1 1],'EdgeColor',colour);
else
    patch((drawOut(1,:)+tipX),(drawOut(2,:)+tipY),colour,'edgecolor',colour);
end

% patchHandle is the handle to the patch drawn,
% shapeExtent is the bouding box of the shape.
function [patchHandle]=drawNode(style,x,y,width,height,edgeColour,bgColour,property,value)
isInverted=false;
extraOutlines=0;
if strmatch('inv', style)
    isInverted=true;
    style=style(4:end);
end
if strmatch('double',style)
    extraOutlines=1;
    style=style(7:end);
elseif strmatch('triple',style)
    extraOutlines=2;
    style=style(7:end);
end
switch lower(style)
    % add defines for other styles here... These are just the base patch
    % that will get transformed later. Shapes should fill the unit square
    % and center at zero. These are scaled/shifted later.
    case 'diamond'
        patchX=[0,0.5,0,-0.5];
        patchY=[0.5,0,-0.5,0];
    case {'box' 'rect' 'rectangle'}
        patchX=[-0.5,0.5,0.5,-0.5];
        patchY=[0.5,0.5,-0.5,-0.5];
    case 'msquare'
        patchX=[-0.5,0.5,0.5,-0.5,-0.5,-0.5,-0.2,0.2,0.5,0.5,0.2,-0.2,-0.5];
        patchY=[0.5,0.5,-0.5,-0.5,-0.2,0.2,0.5,0.5,0.2,-0.2,-0.5,-0.5,-0.2];
    case 'mdiamond'
        patchX=[0,0.5,0,-0.5,-0.3,-0.2,0.2,0.3,0.3,0.2,-0.2,-0.3,-0.3];
        patchY=[0.5,0,-0.5,0,0.2,0.3,0.3,0.2,-0.2,-0.3,-0.3,-0.2,0.2];
    case {'circle' 'ellipse'}
        patchX=-1:0.05:1;
        patchY=[sqrt(1-patchX.^2) -sqrt(1-patchX.^2)];
        patchX=[patchX fliplr(patchX)];
        patchX=patchX.*0.5; patchY=patchY.*0.5;
    case 'mcircle'
        patchX=-1:0.05:1;
        patchY=[sqrt(1-patchX.^2) -sqrt(1-patchX.^2)];
        patchY=repmat(patchY,2,1);
        patchX=repmat(patchX,2,1);
        patchY(2,patchY(2,:)>0.6)=0.6;
        patchY(2,patchY(2,:)<-0.6)=-0.6;
        patchX=[patchX fliplr(patchX)];
        patchX=patchX.*0.5; patchY=patchY.*0.5;
        patchX=patchX'; patchY=patchY';
    case 'point'
        patchX=-1:0.05:1;
        patchY=[sqrt(1-patchX.^2) -sqrt(1-patchX.^2)];
        patchX=[patchX fliplr(patchX)];
        patchX=patchX.*0.5; patchY=patchY.*0.5;
        bgColour=[0 0 0];
    case 'triangle'
        patchX=[-0.5,0.5,0];
        patchY=[-0.5,-0.5,0.5];
    case 'trapezium'
        patchX=[-0.5,0.5,0.3,-0.3];
        patchY=[-0.5,-0.5,0.5,0.5];
    case 'parallelogram'
        patchX=[-0.5,0.3,0.5,-0.3];
        patchY=[-0.5,-0.5,0.5,0.5];
    case 'house'
        patchX=[-0.5,0.5,0.5,0,-0.5];
        patchY=[-0.5,-0.5,0.2,0.5,0.2];
    case 'pentagon'
        patchX=[-0.4,0.4,0.5,0,-0.5];
        patchY=[-0.5,-0.5,0.2,0.5,0.2];
    case 'hexagon'
        patchX=[-0.5,-0.3,0.3,0.5,0.3,-0.3];
        patchY=[0,0.5,0.5,0,-0.5,-0.5];
    case {'polygon' 'septagon'}
        patchX=[-0.2,-0.5,-0.4,0,0.4,0.5,0.2];
        patchY=[-0.5,-0.1,0.3,0.5,0.3,-0.1,-0.5];
    case 'octagon'
        patchX=[0.3,0.5,0.5,0.3,-0.3,-0.5,-0.5,-0.3];
        patchY=[-0.5,-0.3,0.3,0.5,0.5,0.3,-0.3,-0.5];
    case 'tab'
        patchX=[-0.5,-0.5,0.5,0.5,-0.5,-0.5,-0.2,-0.2];
        patchY=[0.35,-0.5,-0.5,0.35,0.35,0.5,0.5,0.35];
        if isequal(bgColour,edgeColour)
            edgeColour=[0 0 0];
        end
    case 'note'
        patchX=[-0.5,-0.5,0.5,0.5 0.3,0.3,0.5,0.3];
        patchY=[0.5,-0.5,-0.5,0.3,0.3,0.5,0.3,0.5];
        if isequal(bgColour,edgeColour)
            edgeColour=[0 0 0];
        end
    case 'folder'
        patchX=[-0.5,-0.5,0.5,0.5 0.4,0.1,0];
        patchY=[0.4,-0.5,-0.5,0.4,0.5,0.5,0.4];
        if isequal(bgColour,edgeColour)
            edgeColour=[0 0 0];
        end
    case 'box3d'
        patchX=[-0.5,-0.5,0.3,0.3;0.3,0.5,0.5,0.3;-0.5,-0.3,0.5,0.3]';
        patchY=[0.3,-0.5,-0.5,0.3;-0.5,-0.3,0.5,0.3;0.3,0.5,0.5,0.3]';
        if isequal(bgColour,edgeColour)
            edgeColour=[0 0 0];
        end
    case 'component'
        patchX=[-0.4,-0.4,0.5,0.5;-0.5,-0.3,-0.3,-0.5;-0.5,-0.3,-0.3,-0.5]';
        patchY=[0.5,-0.5,-0.5,0.5;0.4,0.4,0.25,0.25;-0.4,-0.4,-0.25,-0.25]';
        if isequal(bgColour,edgeColour)
            edgeColour=[0 0 0];
        end
    case {'none' 'plaintext'}
        patchX=[-0.5,0.5,0.5,-0.5];
        patchY=[0.5,0.5,-0.5,-0.5];
        edgeColour='none';
        bgColour='none';
    case 'record'
        error('drawDot:ShapeNotSupported','The record node is not supported because the plain output is not detailed enough to accurately draw such a node.');
    otherwise
        patchX=[-0.5,0.5,0.5,-0.5];
        patchY=[0.5,0.5,-0.5,-0.5];
        edgeColour='none';
        bgColour='none';
        warning('drawDot:ShapeUndefined',['The shape ' style ' is not defined. Please add me to the drawNode function!']);
        return;
end
if isInverted
    patchY=-patchY;
end
patchX=(patchX*width)+x;
patchY=(patchY*height)+y;
if ~isempty(property)
    patchHandle=patch(patchX,patchY,[0 0 0],'FaceColor',bgColour,'EdgeColor',edgeColour,property,value,'lineWidth',extraOutlines+1);
else
    patchHandle=patch(patchX,patchY,[0 0 0],'FaceColor',bgColour,'EdgeColor',edgeColour,'lineWidth',extraOutlines+1);
end


function colours=defineX11Colours()
colours.aliceblue=[0.941176 0.972549 1.000000];
colours.antiquewhite=[0.980392 0.921569 0.843137];
colours.antiquewhite1=[1.000000 0.937255 0.858824];
colours.antiquewhite2=[0.933333 0.874510 0.800000];
colours.antiquewhite3=[0.803922 0.752941 0.690196];
colours.antiquewhite4=[0.545098 0.513725 0.470588];
colours.aquamarine=[0.498039 1.000000 0.831373];
colours.aquamarine1=[0.498039 1.000000 0.831373];
colours.aquamarine2=[0.462745 0.933333 0.776471];
colours.aquamarine3=[0.400000 0.803922 0.666667];
colours.aquamarine4=[0.270588 0.545098 0.454902];
colours.azure=[0.941176 1.000000 1.000000];
colours.azure1=[0.941176 1.000000 1.000000];
colours.azure2=[0.878431 0.933333 0.933333];
colours.azure3=[0.756863 0.803922 0.803922];
colours.azure4=[0.513725 0.545098 0.545098];
colours.beige=[0.960784 0.960784 0.862745];
colours.bisque=[1.000000 0.894118 0.768627];
colours.bisque1=[1.000000 0.894118 0.768627];
colours.bisque2=[0.933333 0.835294 0.717647];
colours.bisque3=[0.803922 0.717647 0.619608];
colours.bisque4=[0.545098 0.490196 0.419608];
colours.black=[0.000000 0.000000 0.000000];
colours.blanchedalmond=[1.000000 0.921569 0.803922];
colours.blue=[0.000000 0.000000 1.000000];
colours.blue1=[0.000000 0.000000 1.000000];
colours.blue2=[0.000000 0.000000 0.933333];
colours.blue3=[0.000000 0.000000 0.803922];
colours.blue4=[0.000000 0.000000 0.545098];
colours.blueviolet=[0.541176 0.168627 0.886275];
colours.brown=[0.647059 0.164706 0.164706];
colours.brown1=[1.000000 0.250980 0.250980];
colours.brown2=[0.933333 0.231373 0.231373];
colours.brown3=[0.803922 0.200000 0.200000];
colours.brown4=[0.545098 0.137255 0.137255];
colours.burlywood=[0.870588 0.721569 0.529412];
colours.burlywood1=[1.000000 0.827451 0.607843];
colours.burlywood2=[0.933333 0.772549 0.568627];
colours.burlywood3=[0.803922 0.666667 0.490196];
colours.burlywood4=[0.545098 0.450980 0.333333];
colours.cadetblue=[0.372549 0.619608 0.627451];
colours.cadetblue1=[0.596078 0.960784 1.000000];
colours.cadetblue2=[0.556863 0.898039 0.933333];
colours.cadetblue3=[0.478431 0.772549 0.803922];
colours.cadetblue4=[0.325490 0.525490 0.545098];
colours.chartreuse=[0.498039 1.000000 0.000000];
colours.chartreuse1=[0.498039 1.000000 0.000000];
colours.chartreuse2=[0.462745 0.933333 0.000000];
colours.chartreuse3=[0.400000 0.803922 0.000000];
colours.chartreuse4=[0.270588 0.545098 0.000000];
colours.chocolate=[0.823529 0.411765 0.117647];
colours.chocolate1=[1.000000 0.498039 0.141176];
colours.chocolate2=[0.933333 0.462745 0.129412];
colours.chocolate3=[0.803922 0.400000 0.113725];
colours.chocolate4=[0.545098 0.270588 0.074510];
colours.coral=[1.000000 0.498039 0.313725];
colours.coral1=[1.000000 0.447059 0.337255];
colours.coral2=[0.933333 0.415686 0.313725];
colours.coral3=[0.803922 0.356863 0.270588];
colours.coral4=[0.545098 0.243137 0.184314];
colours.cornflowerblue=[0.392157 0.584314 0.929412];
colours.cornsilk=[1.000000 0.972549 0.862745];
colours.cornsilk1=[1.000000 0.972549 0.862745];
colours.cornsilk2=[0.933333 0.909804 0.803922];
colours.cornsilk3=[0.803922 0.784314 0.694118];
colours.cornsilk4=[0.545098 0.533333 0.470588];
colours.crimson=[0.862745 0.078431 0.235294];
colours.cyan=[0.000000 1.000000 1.000000];
colours.cyan1=[0.000000 1.000000 1.000000];
colours.cyan2=[0.000000 0.933333 0.933333];
colours.cyan3=[0.000000 0.803922 0.803922];
colours.cyan4=[0.000000 0.545098 0.545098];
colours.darkgoldenrod=[0.721569 0.525490 0.043137];
colours.darkgoldenrod1=[1.000000 0.725490 0.058824];
colours.darkgoldenrod2=[0.933333 0.678431 0.054902];
colours.darkgoldenrod3=[0.803922 0.584314 0.047059];
colours.darkgoldenrod4=[0.545098 0.396078 0.031373];
colours.darkgreen=[0.000000 0.392157 0.000000];
colours.darkkhaki=[0.741176 0.717647 0.419608];
colours.darkolivegreen=[0.333333 0.419608 0.184314];
colours.darkolivegreen1=[0.792157 1.000000 0.439216];
colours.darkolivegreen2=[0.737255 0.933333 0.407843];
colours.darkolivegreen3=[0.635294 0.803922 0.352941];
colours.darkolivegreen4=[0.431373 0.545098 0.239216];
colours.darkorange=[1.000000 0.549020 0.000000];
colours.darkorange1=[1.000000 0.498039 0.000000];
colours.darkorange2=[0.933333 0.462745 0.000000];
colours.darkorange3=[0.803922 0.400000 0.000000];
colours.darkorange4=[0.545098 0.270588 0.000000];
colours.darkorchid=[0.600000 0.196078 0.800000];
colours.darkorchid1=[0.749020 0.243137 1.000000];
colours.darkorchid2=[0.698039 0.227451 0.933333];
colours.darkorchid3=[0.603922 0.196078 0.803922];
colours.darkorchid4=[0.407843 0.133333 0.545098];
colours.darksalmon=[0.913725 0.588235 0.478431];
colours.darkseagreen=[0.560784 0.737255 0.560784];
colours.darkseagreen1=[0.756863 1.000000 0.756863];
colours.darkseagreen2=[0.705882 0.933333 0.705882];
colours.darkseagreen3=[0.607843 0.803922 0.607843];
colours.darkseagreen4=[0.411765 0.545098 0.411765];
colours.darkslateblue=[0.282353 0.239216 0.545098];
colours.darkslategray=[0.184314 0.309804 0.309804];
colours.darkslategray1=[0.592157 1.000000 1.000000];
colours.darkslategray2=[0.552941 0.933333 0.933333];
colours.darkslategray3=[0.474510 0.803922 0.803922];
colours.darkslategray4=[0.321569 0.545098 0.545098];
colours.darkslategrey=[0.184314 0.309804 0.309804];
colours.darkturquoise=[0.000000 0.807843 0.819608];
colours.darkviolet=[0.580392 0.000000 0.827451];
colours.deeppink=[1.000000 0.078431 0.576471];
colours.deeppink1=[1.000000 0.078431 0.576471];
colours.deeppink2=[0.933333 0.070588 0.537255];
colours.deeppink3=[0.803922 0.062745 0.462745];
colours.deeppink4=[0.545098 0.039216 0.313725];
colours.deepskyblue=[0.000000 0.749020 1.000000];
colours.deepskyblue1=[0.000000 0.749020 1.000000];
colours.deepskyblue2=[0.000000 0.698039 0.933333];
colours.deepskyblue3=[0.000000 0.603922 0.803922];
colours.deepskyblue4=[0.000000 0.407843 0.545098];
colours.dimgray=[0.411765 0.411765 0.411765];
colours.dimgrey=[0.411765 0.411765 0.411765];
colours.dodgerblue=[0.117647 0.564706 1.000000];
colours.dodgerblue1=[0.117647 0.564706 1.000000];
colours.dodgerblue2=[0.109804 0.525490 0.933333];
colours.dodgerblue3=[0.094118 0.454902 0.803922];
colours.dodgerblue4=[0.062745 0.305882 0.545098];
colours.firebrick=[0.698039 0.133333 0.133333];
colours.firebrick1=[1.000000 0.188235 0.188235];
colours.firebrick2=[0.933333 0.172549 0.172549];
colours.firebrick3=[0.803922 0.149020 0.149020];
colours.firebrick4=[0.545098 0.101961 0.101961];
colours.floralwhite=[1.000000 0.980392 0.941176];
colours.forestgreen=[0.133333 0.545098 0.133333];
colours.gainsboro=[0.862745 0.862745 0.862745];
colours.ghostwhite=[0.972549 0.972549 1.000000];
colours.gold=[1.000000 0.843137 0.000000];
colours.gold1=[1.000000 0.843137 0.000000];
colours.gold2=[0.933333 0.788235 0.000000];
colours.gold3=[0.803922 0.678431 0.000000];
colours.gold4=[0.545098 0.458824 0.000000];
colours.goldenrod=[0.854902 0.647059 0.125490];
colours.goldenrod1=[1.000000 0.756863 0.145098];
colours.goldenrod2=[0.933333 0.705882 0.133333];
colours.goldenrod3=[0.803922 0.607843 0.113725];
colours.goldenrod4=[0.545098 0.411765 0.078431];
colours.gray=[0.752941 0.752941 0.752941];
colours.gray0=[0.000000 0.000000 0.000000];
colours.gray1=[0.011765 0.011765 0.011765];
colours.gray2=[0.019608 0.019608 0.019608];
colours.gray3=[0.031373 0.031373 0.031373];
colours.gray4=[0.039216 0.039216 0.039216];
colours.gray5=[0.050980 0.050980 0.050980];
colours.gray6=[0.058824 0.058824 0.058824];
colours.gray7=[0.070588 0.070588 0.070588];
colours.gray8=[0.078431 0.078431 0.078431];
colours.gray9=[0.090196 0.090196 0.090196];
colours.gray10=[0.101961 0.101961 0.101961];
colours.gray11=[0.109804 0.109804 0.109804];
colours.gray12=[0.121569 0.121569 0.121569];
colours.gray13=[0.129412 0.129412 0.129412];
colours.gray14=[0.141176 0.141176 0.141176];
colours.gray15=[0.149020 0.149020 0.149020];
colours.gray16=[0.160784 0.160784 0.160784];
colours.gray17=[0.168627 0.168627 0.168627];
colours.gray18=[0.180392 0.180392 0.180392];
colours.gray19=[0.188235 0.188235 0.188235];
colours.gray20=[0.200000 0.200000 0.200000];
colours.gray21=[0.211765 0.211765 0.211765];
colours.gray22=[0.219608 0.219608 0.219608];
colours.gray23=[0.231373 0.231373 0.231373];
colours.gray24=[0.239216 0.239216 0.239216];
colours.gray25=[0.250980 0.250980 0.250980];
colours.gray26=[0.258824 0.258824 0.258824];
colours.gray27=[0.270588 0.270588 0.270588];
colours.gray28=[0.278431 0.278431 0.278431];
colours.gray29=[0.290196 0.290196 0.290196];
colours.gray30=[0.301961 0.301961 0.301961];
colours.gray31=[0.309804 0.309804 0.309804];
colours.gray32=[0.321569 0.321569 0.321569];
colours.gray33=[0.329412 0.329412 0.329412];
colours.gray34=[0.341176 0.341176 0.341176];
colours.gray35=[0.349020 0.349020 0.349020];
colours.gray36=[0.360784 0.360784 0.360784];
colours.gray37=[0.368627 0.368627 0.368627];
colours.gray38=[0.380392 0.380392 0.380392];
colours.gray39=[0.388235 0.388235 0.388235];
colours.gray40=[0.400000 0.400000 0.400000];
colours.gray41=[0.411765 0.411765 0.411765];
colours.gray42=[0.419608 0.419608 0.419608];
colours.gray43=[0.431373 0.431373 0.431373];
colours.gray44=[0.439216 0.439216 0.439216];
colours.gray45=[0.450980 0.450980 0.450980];
colours.gray46=[0.458824 0.458824 0.458824];
colours.gray47=[0.470588 0.470588 0.470588];
colours.gray48=[0.478431 0.478431 0.478431];
colours.gray49=[0.490196 0.490196 0.490196];
colours.gray50=[0.498039 0.498039 0.498039];
colours.gray51=[0.509804 0.509804 0.509804];
colours.gray52=[0.521569 0.521569 0.521569];
colours.gray53=[0.529412 0.529412 0.529412];
colours.gray54=[0.541176 0.541176 0.541176];
colours.gray55=[0.549020 0.549020 0.549020];
colours.gray56=[0.560784 0.560784 0.560784];
colours.gray57=[0.568627 0.568627 0.568627];
colours.gray58=[0.580392 0.580392 0.580392];
colours.gray59=[0.588235 0.588235 0.588235];
colours.gray60=[0.600000 0.600000 0.600000];
colours.gray61=[0.611765 0.611765 0.611765];
colours.gray62=[0.619608 0.619608 0.619608];
colours.gray63=[0.631373 0.631373 0.631373];
colours.gray64=[0.639216 0.639216 0.639216];
colours.gray65=[0.650980 0.650980 0.650980];
colours.gray66=[0.658824 0.658824 0.658824];
colours.gray67=[0.670588 0.670588 0.670588];
colours.gray68=[0.678431 0.678431 0.678431];
colours.gray69=[0.690196 0.690196 0.690196];
colours.gray70=[0.701961 0.701961 0.701961];
colours.gray71=[0.709804 0.709804 0.709804];
colours.gray72=[0.721569 0.721569 0.721569];
colours.gray73=[0.729412 0.729412 0.729412];
colours.gray74=[0.741176 0.741176 0.741176];
colours.gray75=[0.749020 0.749020 0.749020];
colours.gray76=[0.760784 0.760784 0.760784];
colours.gray77=[0.768627 0.768627 0.768627];
colours.gray78=[0.780392 0.780392 0.780392];
colours.gray79=[0.788235 0.788235 0.788235];
colours.gray80=[0.800000 0.800000 0.800000];
colours.gray81=[0.811765 0.811765 0.811765];
colours.gray82=[0.819608 0.819608 0.819608];
colours.gray83=[0.831373 0.831373 0.831373];
colours.gray84=[0.839216 0.839216 0.839216];
colours.gray85=[0.850980 0.850980 0.850980];
colours.gray86=[0.858824 0.858824 0.858824];
colours.gray87=[0.870588 0.870588 0.870588];
colours.gray88=[0.878431 0.878431 0.878431];
colours.gray89=[0.890196 0.890196 0.890196];
colours.gray90=[0.898039 0.898039 0.898039];
colours.gray91=[0.909804 0.909804 0.909804];
colours.gray92=[0.921569 0.921569 0.921569];
colours.gray93=[0.929412 0.929412 0.929412];
colours.gray94=[0.941176 0.941176 0.941176];
colours.gray95=[0.949020 0.949020 0.949020];
colours.gray96=[0.960784 0.960784 0.960784];
colours.gray97=[0.968627 0.968627 0.968627];
colours.gray98=[0.980392 0.980392 0.980392];
colours.gray99=[0.988235 0.988235 0.988235];
colours.gray100=[1.000000 1.000000 1.000000];
colours.green=[0.000000 1.000000 0.000000];
colours.green1=[0.000000 1.000000 0.000000];
colours.green2=[0.000000 0.933333 0.000000];
colours.green3=[0.000000 0.803922 0.000000];
colours.green4=[0.000000 0.545098 0.000000];
colours.greenyellow=[0.678431 1.000000 0.184314];
colours.grey=[0.752941 0.752941 0.752941];
colours.grey0=[0.000000 0.000000 0.000000];
colours.grey1=[0.011765 0.011765 0.011765];
colours.grey2=[0.019608 0.019608 0.019608];
colours.grey3=[0.031373 0.031373 0.031373];
colours.grey4=[0.039216 0.039216 0.039216];
colours.grey5=[0.050980 0.050980 0.050980];
colours.grey6=[0.058824 0.058824 0.058824];
colours.grey7=[0.070588 0.070588 0.070588];
colours.grey8=[0.078431 0.078431 0.078431];
colours.grey9=[0.090196 0.090196 0.090196];
colours.grey10=[0.101961 0.101961 0.101961];
colours.grey11=[0.109804 0.109804 0.109804];
colours.grey12=[0.121569 0.121569 0.121569];
colours.grey13=[0.129412 0.129412 0.129412];
colours.grey14=[0.141176 0.141176 0.141176];
colours.grey15=[0.149020 0.149020 0.149020];
colours.grey16=[0.160784 0.160784 0.160784];
colours.grey17=[0.168627 0.168627 0.168627];
colours.grey18=[0.180392 0.180392 0.180392];
colours.grey19=[0.188235 0.188235 0.188235];
colours.grey20=[0.200000 0.200000 0.200000];
colours.grey21=[0.211765 0.211765 0.211765];
colours.grey22=[0.219608 0.219608 0.219608];
colours.grey23=[0.231373 0.231373 0.231373];
colours.grey24=[0.239216 0.239216 0.239216];
colours.grey25=[0.250980 0.250980 0.250980];
colours.grey26=[0.258824 0.258824 0.258824];
colours.grey27=[0.270588 0.270588 0.270588];
colours.grey28=[0.278431 0.278431 0.278431];
colours.grey29=[0.290196 0.290196 0.290196];
colours.grey30=[0.301961 0.301961 0.301961];
colours.grey31=[0.309804 0.309804 0.309804];
colours.grey32=[0.321569 0.321569 0.321569];
colours.grey33=[0.329412 0.329412 0.329412];
colours.grey34=[0.341176 0.341176 0.341176];
colours.grey35=[0.349020 0.349020 0.349020];
colours.grey36=[0.360784 0.360784 0.360784];
colours.grey37=[0.368627 0.368627 0.368627];
colours.grey38=[0.380392 0.380392 0.380392];
colours.grey39=[0.388235 0.388235 0.388235];
colours.grey40=[0.400000 0.400000 0.400000];
colours.grey41=[0.411765 0.411765 0.411765];
colours.grey42=[0.419608 0.419608 0.419608];
colours.grey43=[0.431373 0.431373 0.431373];
colours.grey44=[0.439216 0.439216 0.439216];
colours.grey45=[0.450980 0.450980 0.450980];
colours.grey46=[0.458824 0.458824 0.458824];
colours.grey47=[0.470588 0.470588 0.470588];
colours.grey48=[0.478431 0.478431 0.478431];
colours.grey49=[0.490196 0.490196 0.490196];
colours.grey50=[0.498039 0.498039 0.498039];
colours.grey51=[0.509804 0.509804 0.509804];
colours.grey52=[0.521569 0.521569 0.521569];
colours.grey53=[0.529412 0.529412 0.529412];
colours.grey54=[0.541176 0.541176 0.541176];
colours.grey55=[0.549020 0.549020 0.549020];
colours.grey56=[0.560784 0.560784 0.560784];
colours.grey57=[0.568627 0.568627 0.568627];
colours.grey58=[0.580392 0.580392 0.580392];
colours.grey59=[0.588235 0.588235 0.588235];
colours.grey60=[0.600000 0.600000 0.600000];
colours.grey61=[0.611765 0.611765 0.611765];
colours.grey62=[0.619608 0.619608 0.619608];
colours.grey63=[0.631373 0.631373 0.631373];
colours.grey64=[0.639216 0.639216 0.639216];
colours.grey65=[0.650980 0.650980 0.650980];
colours.grey66=[0.658824 0.658824 0.658824];
colours.grey67=[0.670588 0.670588 0.670588];
colours.grey68=[0.678431 0.678431 0.678431];
colours.grey69=[0.690196 0.690196 0.690196];
colours.grey70=[0.701961 0.701961 0.701961];
colours.grey71=[0.709804 0.709804 0.709804];
colours.grey72=[0.721569 0.721569 0.721569];
colours.grey73=[0.729412 0.729412 0.729412];
colours.grey74=[0.741176 0.741176 0.741176];
colours.grey75=[0.749020 0.749020 0.749020];
colours.grey76=[0.760784 0.760784 0.760784];
colours.grey77=[0.768627 0.768627 0.768627];
colours.grey78=[0.780392 0.780392 0.780392];
colours.grey79=[0.788235 0.788235 0.788235];
colours.grey80=[0.800000 0.800000 0.800000];
colours.grey81=[0.811765 0.811765 0.811765];
colours.grey82=[0.819608 0.819608 0.819608];
colours.grey83=[0.831373 0.831373 0.831373];
colours.grey84=[0.839216 0.839216 0.839216];
colours.grey85=[0.850980 0.850980 0.850980];
colours.grey86=[0.858824 0.858824 0.858824];
colours.grey87=[0.870588 0.870588 0.870588];
colours.grey88=[0.878431 0.878431 0.878431];
colours.grey89=[0.890196 0.890196 0.890196];
colours.grey90=[0.898039 0.898039 0.898039];
colours.grey91=[0.909804 0.909804 0.909804];
colours.grey92=[0.921569 0.921569 0.921569];
colours.grey93=[0.929412 0.929412 0.929412];
colours.grey94=[0.941176 0.941176 0.941176];
colours.grey95=[0.949020 0.949020 0.949020];
colours.grey96=[0.960784 0.960784 0.960784];
colours.grey97=[0.968627 0.968627 0.968627];
colours.grey98=[0.980392 0.980392 0.980392];
colours.grey99=[0.988235 0.988235 0.988235];
colours.grey100=[1.000000 1.000000 1.000000];
colours.honeydew=[0.941176 1.000000 0.941176];
colours.honeydew1=[0.941176 1.000000 0.941176];
colours.honeydew2=[0.878431 0.933333 0.878431];
colours.honeydew3=[0.756863 0.803922 0.756863];
colours.honeydew4=[0.513725 0.545098 0.513725];
colours.hotpink=[1.000000 0.411765 0.705882];
colours.hotpink1=[1.000000 0.431373 0.705882];
colours.hotpink2=[0.933333 0.415686 0.654902];
colours.hotpink3=[0.803922 0.376471 0.564706];
colours.hotpink4=[0.545098 0.227451 0.384314];
colours.indianred=[0.803922 0.360784 0.360784];
colours.indianred1=[1.000000 0.415686 0.415686];
colours.indianred2=[0.933333 0.388235 0.388235];
colours.indianred3=[0.803922 0.333333 0.333333];
colours.indianred4=[0.545098 0.227451 0.227451];
colours.indigo=[0.294118 0.000000 0.509804];
colours.ivory=[1.000000 1.000000 0.941176];
colours.ivory1=[1.000000 1.000000 0.941176];
colours.ivory2=[0.933333 0.933333 0.878431];
colours.ivory3=[0.803922 0.803922 0.756863];
colours.ivory4=[0.545098 0.545098 0.513725];
colours.khaki=[0.941176 0.901961 0.549020];
colours.khaki1=[1.000000 0.964706 0.560784];
colours.khaki2=[0.933333 0.901961 0.521569];
colours.khaki3=[0.803922 0.776471 0.450980];
colours.khaki4=[0.545098 0.525490 0.305882];
colours.lavender=[0.901961 0.901961 0.980392];
colours.lavenderblush=[1.000000 0.941176 0.960784];
colours.lavenderblush1=[1.000000 0.941176 0.960784];
colours.lavenderblush2=[0.933333 0.878431 0.898039];
colours.lavenderblush3=[0.803922 0.756863 0.772549];
colours.lavenderblush4=[0.545098 0.513725 0.525490];
colours.lawngreen=[0.486275 0.988235 0.000000];
colours.lemonchiffon=[1.000000 0.980392 0.803922];
colours.lemonchiffon1=[1.000000 0.980392 0.803922];
colours.lemonchiffon2=[0.933333 0.913725 0.749020];
colours.lemonchiffon3=[0.803922 0.788235 0.647059];
colours.lemonchiffon4=[0.545098 0.537255 0.439216];
colours.lightblue=[0.678431 0.847059 0.901961];
colours.lightblue1=[0.749020 0.937255 1.000000];
colours.lightblue2=[0.698039 0.874510 0.933333];
colours.lightblue3=[0.603922 0.752941 0.803922];
colours.lightblue4=[0.407843 0.513725 0.545098];
colours.lightcoral=[0.941176 0.501961 0.501961];
colours.lightcyan=[0.878431 1.000000 1.000000];
colours.lightcyan1=[0.878431 1.000000 1.000000];
colours.lightcyan2=[0.819608 0.933333 0.933333];
colours.lightcyan3=[0.705882 0.803922 0.803922];
colours.lightcyan4=[0.478431 0.545098 0.545098];
colours.lightgoldenrod=[0.933333 0.866667 0.509804];
colours.lightgoldenrod1=[1.000000 0.925490 0.545098];
colours.lightgoldenrod2=[0.933333 0.862745 0.509804];
colours.lightgoldenrod3=[0.803922 0.745098 0.439216];
colours.lightgoldenrod4=[0.545098 0.505882 0.298039];
colours.lightgoldenrodyellow=[0.980392 0.980392 0.823529];
colours.lightgray=[0.827451 0.827451 0.827451];
colours.lightgrey=[0.827451 0.827451 0.827451];
colours.lightpink=[1.000000 0.713725 0.756863];
colours.lightpink1=[1.000000 0.682353 0.725490];
colours.lightpink2=[0.933333 0.635294 0.678431];
colours.lightpink3=[0.803922 0.549020 0.584314];
colours.lightpink4=[0.545098 0.372549 0.396078];
colours.lightsalmon=[1.000000 0.627451 0.478431];
colours.lightsalmon1=[1.000000 0.627451 0.478431];
colours.lightsalmon2=[0.933333 0.584314 0.447059];
colours.lightsalmon3=[0.803922 0.505882 0.384314];
colours.lightsalmon4=[0.545098 0.341176 0.258824];
colours.lightseagreen=[0.125490 0.698039 0.666667];
colours.lightskyblue=[0.529412 0.807843 0.980392];
colours.lightskyblue1=[0.690196 0.886275 1.000000];
colours.lightskyblue2=[0.643137 0.827451 0.933333];
colours.lightskyblue3=[0.552941 0.713725 0.803922];
colours.lightskyblue4=[0.376471 0.482353 0.545098];
colours.lightslateblue=[0.517647 0.439216 1.000000];
colours.lightslategray=[0.466667 0.533333 0.600000];
colours.lightslategrey=[0.466667 0.533333 0.600000];
colours.lightsteelblue=[0.690196 0.768627 0.870588];
colours.lightsteelblue1=[0.792157 0.882353 1.000000];
colours.lightsteelblue2=[0.737255 0.823529 0.933333];
colours.lightsteelblue3=[0.635294 0.709804 0.803922];
colours.lightsteelblue4=[0.431373 0.482353 0.545098];
colours.lightyellow=[1.000000 1.000000 0.878431];
colours.lightyellow1=[1.000000 1.000000 0.878431];
colours.lightyellow2=[0.933333 0.933333 0.819608];
colours.lightyellow3=[0.803922 0.803922 0.705882];
colours.lightyellow4=[0.545098 0.545098 0.478431];
colours.limegreen=[0.196078 0.803922 0.196078];
colours.linen=[0.980392 0.941176 0.901961];
colours.magenta=[1.000000 0.000000 1.000000];
colours.magenta1=[1.000000 0.000000 1.000000];
colours.magenta2=[0.933333 0.000000 0.933333];
colours.magenta3=[0.803922 0.000000 0.803922];
colours.magenta4=[0.545098 0.000000 0.545098];
colours.maroon=[0.690196 0.188235 0.376471];
colours.maroon1=[1.000000 0.203922 0.701961];
colours.maroon2=[0.933333 0.188235 0.654902];
colours.maroon3=[0.803922 0.160784 0.564706];
colours.maroon4=[0.545098 0.109804 0.384314];
colours.mediumaquamarine=[0.400000 0.803922 0.666667];
colours.mediumblue=[0.000000 0.000000 0.803922];
colours.mediumorchid=[0.729412 0.333333 0.827451];
colours.mediumorchid1=[0.878431 0.400000 1.000000];
colours.mediumorchid2=[0.819608 0.372549 0.933333];
colours.mediumorchid3=[0.705882 0.321569 0.803922];
colours.mediumorchid4=[0.478431 0.215686 0.545098];
colours.mediumpurple=[0.576471 0.439216 0.858824];
colours.mediumpurple1=[0.670588 0.509804 1.000000];
colours.mediumpurple2=[0.623529 0.474510 0.933333];
colours.mediumpurple3=[0.537255 0.407843 0.803922];
colours.mediumpurple4=[0.364706 0.278431 0.545098];
colours.mediumseagreen=[0.235294 0.701961 0.443137];
colours.mediumslateblue=[0.482353 0.407843 0.933333];
colours.mediumspringgreen=[0.000000 0.980392 0.603922];
colours.mediumturquoise=[0.282353 0.819608 0.800000];
colours.mediumvioletred=[0.780392 0.082353 0.521569];
colours.midnightblue=[0.098039 0.098039 0.439216];
colours.mintcream=[0.960784 1.000000 0.980392];
colours.mistyrose=[1.000000 0.894118 0.882353];
colours.mistyrose1=[1.000000 0.894118 0.882353];
colours.mistyrose2=[0.933333 0.835294 0.823529];
colours.mistyrose3=[0.803922 0.717647 0.709804];
colours.mistyrose4=[0.545098 0.490196 0.482353];
colours.moccasin=[1.000000 0.894118 0.709804];
colours.navajowhite=[1.000000 0.870588 0.678431];
colours.navajowhite1=[1.000000 0.870588 0.678431];
colours.navajowhite2=[0.933333 0.811765 0.631373];
colours.navajowhite3=[0.803922 0.701961 0.545098];
colours.navajowhite4=[0.545098 0.474510 0.368627];
colours.navy=[0.000000 0.000000 0.501961];
colours.navyblue=[0.000000 0.000000 0.501961];
colours.oldlace=[0.992157 0.960784 0.901961];
colours.olivedrab=[0.419608 0.556863 0.137255];
colours.olivedrab1=[0.752941 1.000000 0.243137];
colours.olivedrab2=[0.701961 0.933333 0.227451];
colours.olivedrab3=[0.603922 0.803922 0.196078];
colours.olivedrab4=[0.411765 0.545098 0.133333];
colours.orange=[1.000000 0.647059 0.000000];
colours.orange1=[1.000000 0.647059 0.000000];
colours.orange2=[0.933333 0.603922 0.000000];
colours.orange3=[0.803922 0.521569 0.000000];
colours.orange4=[0.545098 0.352941 0.000000];
colours.orangered=[1.000000 0.270588 0.000000];
colours.orangered1=[1.000000 0.270588 0.000000];
colours.orangered2=[0.933333 0.250980 0.000000];
colours.orangered3=[0.803922 0.215686 0.000000];
colours.orangered4=[0.545098 0.145098 0.000000];
colours.orchid=[0.854902 0.439216 0.839216];
colours.orchid1=[1.000000 0.513725 0.980392];
colours.orchid2=[0.933333 0.478431 0.913725];
colours.orchid3=[0.803922 0.411765 0.788235];
colours.orchid4=[0.545098 0.278431 0.537255];
colours.palegoldenrod=[0.933333 0.909804 0.666667];
colours.palegreen=[0.596078 0.984314 0.596078];
colours.palegreen1=[0.603922 1.000000 0.603922];
colours.palegreen2=[0.564706 0.933333 0.564706];
colours.palegreen3=[0.486275 0.803922 0.486275];
colours.palegreen4=[0.329412 0.545098 0.329412];
colours.paleturquoise=[0.686275 0.933333 0.933333];
colours.paleturquoise1=[0.733333 1.000000 1.000000];
colours.paleturquoise2=[0.682353 0.933333 0.933333];
colours.paleturquoise3=[0.588235 0.803922 0.803922];
colours.paleturquoise4=[0.400000 0.545098 0.545098];
colours.palevioletred=[0.858824 0.439216 0.576471];
colours.palevioletred1=[1.000000 0.509804 0.670588];
colours.palevioletred2=[0.933333 0.474510 0.623529];
colours.palevioletred3=[0.803922 0.407843 0.537255];
colours.palevioletred4=[0.545098 0.278431 0.364706];
colours.papayawhip=[1.000000 0.937255 0.835294];
colours.peachpuff=[1.000000 0.854902 0.725490];
colours.peachpuff1=[1.000000 0.854902 0.725490];
colours.peachpuff2=[0.933333 0.796078 0.678431];
colours.peachpuff3=[0.803922 0.686275 0.584314];
colours.peachpuff4=[0.545098 0.466667 0.396078];
colours.peru=[0.803922 0.521569 0.247059];
colours.pink=[1.000000 0.752941 0.796078];
colours.pink1=[1.000000 0.709804 0.772549];
colours.pink2=[0.933333 0.662745 0.721569];
colours.pink3=[0.803922 0.568627 0.619608];
colours.pink4=[0.545098 0.388235 0.423529];
colours.plum=[0.866667 0.627451 0.866667];
colours.plum1=[1.000000 0.733333 1.000000];
colours.plum2=[0.933333 0.682353 0.933333];
colours.plum3=[0.803922 0.588235 0.803922];
colours.plum4=[0.545098 0.400000 0.545098];
colours.powderblue=[0.690196 0.878431 0.901961];
colours.purple=[0.627451 0.125490 0.941176];
colours.purple1=[0.607843 0.188235 1.000000];
colours.purple2=[0.568627 0.172549 0.933333];
colours.purple3=[0.490196 0.149020 0.803922];
colours.purple4=[0.333333 0.101961 0.545098];
colours.red=[1.000000 0.000000 0.000000];
colours.red1=[1.000000 0.000000 0.000000];
colours.red2=[0.933333 0.000000 0.000000];
colours.red3=[0.803922 0.000000 0.000000];
colours.red4=[0.545098 0.000000 0.000000];
colours.rosybrown=[0.737255 0.560784 0.560784];
colours.rosybrown1=[1.000000 0.756863 0.756863];
colours.rosybrown2=[0.933333 0.705882 0.705882];
colours.rosybrown3=[0.803922 0.607843 0.607843];
colours.rosybrown4=[0.545098 0.411765 0.411765];
colours.royalblue=[0.254902 0.411765 0.882353];
colours.royalblue1=[0.282353 0.462745 1.000000];
colours.royalblue2=[0.262745 0.431373 0.933333];
colours.royalblue3=[0.227451 0.372549 0.803922];
colours.royalblue4=[0.152941 0.250980 0.545098];
colours.saddlebrown=[0.545098 0.270588 0.074510];
colours.salmon=[0.980392 0.501961 0.447059];
colours.salmon1=[1.000000 0.549020 0.411765];
colours.salmon2=[0.933333 0.509804 0.384314];
colours.salmon3=[0.803922 0.439216 0.329412];
colours.salmon4=[0.545098 0.298039 0.223529];
colours.sandybrown=[0.956863 0.643137 0.376471];
colours.seagreen=[0.180392 0.545098 0.341176];
colours.seagreen1=[0.329412 1.000000 0.623529];
colours.seagreen2=[0.305882 0.933333 0.580392];
colours.seagreen3=[0.262745 0.803922 0.501961];
colours.seagreen4=[0.180392 0.545098 0.341176];
colours.seashell=[1.000000 0.960784 0.933333];
colours.seashell1=[1.000000 0.960784 0.933333];
colours.seashell2=[0.933333 0.898039 0.870588];
colours.seashell3=[0.803922 0.772549 0.749020];
colours.seashell4=[0.545098 0.525490 0.509804];
colours.sienna=[0.627451 0.321569 0.176471];
colours.sienna1=[1.000000 0.509804 0.278431];
colours.sienna2=[0.933333 0.474510 0.258824];
colours.sienna3=[0.803922 0.407843 0.223529];
colours.sienna4=[0.545098 0.278431 0.149020];
colours.skyblue=[0.529412 0.807843 0.921569];
colours.skyblue1=[0.529412 0.807843 1.000000];
colours.skyblue2=[0.494118 0.752941 0.933333];
colours.skyblue3=[0.423529 0.650980 0.803922];
colours.skyblue4=[0.290196 0.439216 0.545098];
colours.slateblue=[0.415686 0.352941 0.803922];
colours.slateblue1=[0.513725 0.435294 1.000000];
colours.slateblue2=[0.478431 0.403922 0.933333];
colours.slateblue3=[0.411765 0.349020 0.803922];
colours.slateblue4=[0.278431 0.235294 0.545098];
colours.slategray=[0.439216 0.501961 0.564706];
colours.slategray1=[0.776471 0.886275 1.000000];
colours.slategray2=[0.725490 0.827451 0.933333];
colours.slategray3=[0.623529 0.713725 0.803922];
colours.slategray4=[0.423529 0.482353 0.545098];
colours.slategrey=[0.439216 0.501961 0.564706];
colours.snow=[1.000000 0.980392 0.980392];
colours.snow1=[1.000000 0.980392 0.980392];
colours.snow2=[0.933333 0.913725 0.913725];
colours.snow3=[0.803922 0.788235 0.788235];
colours.snow4=[0.545098 0.537255 0.537255];
colours.springgreen=[0.000000 1.000000 0.498039];
colours.springgreen1=[0.000000 1.000000 0.498039];
colours.springgreen2=[0.000000 0.933333 0.462745];
colours.springgreen3=[0.000000 0.803922 0.400000];
colours.springgreen4=[0.000000 0.545098 0.270588];
colours.steelblue=[0.274510 0.509804 0.705882];
colours.steelblue1=[0.388235 0.721569 1.000000];
colours.steelblue2=[0.360784 0.674510 0.933333];
colours.steelblue3=[0.309804 0.580392 0.803922];
colours.steelblue4=[0.211765 0.392157 0.545098];
colours.tan=[0.823529 0.705882 0.549020];
colours.tan1=[1.000000 0.647059 0.309804];
colours.tan2=[0.933333 0.603922 0.286275];
colours.tan3=[0.803922 0.521569 0.247059];
colours.tan4=[0.545098 0.352941 0.168627];
colours.thistle=[0.847059 0.749020 0.847059];
colours.thistle1=[1.000000 0.882353 1.000000];
colours.thistle2=[0.933333 0.823529 0.933333];
colours.thistle3=[0.803922 0.709804 0.803922];
colours.thistle4=[0.545098 0.482353 0.545098];
colours.tomato=[1.000000 0.388235 0.278431];
colours.tomato1=[1.000000 0.388235 0.278431];
colours.tomato2=[0.933333 0.360784 0.258824];
colours.tomato3=[0.803922 0.309804 0.223529];
colours.tomato4=[0.545098 0.211765 0.149020];
colours.transparent=[1.000000 1.000000 0.996078];
colours.turquoise=[0.250980 0.878431 0.815686];
colours.turquoise1=[0.000000 0.960784 1.000000];
colours.turquoise2=[0.000000 0.898039 0.933333];
colours.turquoise3=[0.000000 0.772549 0.803922];
colours.turquoise4=[0.000000 0.525490 0.545098];
colours.violet=[0.933333 0.509804 0.933333];
colours.violetred=[0.815686 0.125490 0.564706];
colours.violetred1=[1.000000 0.243137 0.588235];
colours.violetred2=[0.933333 0.227451 0.549020];
colours.violetred3=[0.803922 0.196078 0.470588];
colours.violetred4=[0.545098 0.133333 0.321569];
colours.wheat=[0.960784 0.870588 0.701961];
colours.wheat1=[1.000000 0.905882 0.729412];
colours.wheat2=[0.933333 0.847059 0.682353];
colours.wheat3=[0.803922 0.729412 0.588235];
colours.wheat4=[0.545098 0.494118 0.400000];
colours.white=[1.000000 1.000000 1.000000];
colours.whitesmoke=[0.960784 0.960784 0.960784];
colours.yellow=[1.000000 1.000000 0.000000];
colours.yellow1=[1.000000 1.000000 0.000000];
colours.yellow2=[0.933333 0.933333 0.000000];
colours.yellow3=[0.803922 0.803922 0.000000];
colours.yellow4=[0.545098 0.545098 0.000000];
colours.yellowgreen=[0.603922 0.803922 0.196078];
