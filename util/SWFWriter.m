classdef SWFWriter < hgsetget
% Implements export to Adobe SWF (vectorized movie graphics).
% I would have loved to copy the interface to VideoWriter, but found it a
% poor fit.  So I mimic the (now outdated) interface to avifile instead.

properties
  fps = 30;        % frames per second
  poster = true;  % set to true to have a pdf of the first frame saved with the same filename (+.pdf).  
  loop = false;    % set to true to have it loop
end

properties (SetAccess=private) 
  filename;
  frames = 0;
  dirname=[];
  
  width=[]; height=[];
end

properties (SetAccess=private,GetAccess=public)
  bbox;  % set automatically on write (useful for referencing the bbox in the future)
end

methods
  function obj = SWFWriter(filename)
    if (nargin<1)
      [filename,pathname,filterindex] = uiputfile('*.swf','Save playback to SWF');
      % check for cancel button or dialog close
      if (filterindex <= 0)
          % user canceled or closed the dialog
          error('User canceled SWF creation.');
      end
      filename = fullfile(pathname,filename);
    end
    
    % remove .swf if it's on there (it will be added later)
    ind = strfind(lower(filename),'.swf');
    if (~isempty(ind))
      filename=filename(1:ind(end)-1);
    end
    
    obj.filename = filename;
    
    % now open up directory for temp files
    obj.dirname = [filename,'_frames'];
    try rmdir(obj.dirname,'s'); catch end % zap directory if it exists
    mkdir(obj.dirname);
  end
  
  function obj = close(obj)
    % now write the actual SWF file

    if (obj.frames<1) 
      error('no swf frames to write'); 
    end
    
    % grab the bounding boxes from all eps files
    system(['more ', obj.dirname, '/*.eps | grep BoundingBox | sed -e "s/.*://g" | grep -v "(atend)" > ',obj.dirname,'/bbox.txt']);
    bbox = dlmread([obj.dirname,'/bbox.txt']);
    obj.bbox=[min(bbox(:,1:2)),max(bbox(:,3:4))];
    
    % set common bounding box
    %cmd{1}=['ls ', obj.dirname,'/*.eps | xargs -P 8 sed -e "s/BoundingBox:.*$/BoundingBox: ',num2str(obj.bbox),'/g" -i ".bk"'];
    cmd{1}=['ls ', obj.dirname,'/*.eps | xargs -P 8 sed -i -e "s/BoundingBox:.*$/BoundingBox: ',num2str(obj.bbox),'/g"'];
    
    % convert eps to pdf
    cmd{2}=['ls ', obj.dirname,'/*.eps | xargs -P 8 -n1 epstopdf'];

    % merge pdfs
    cmd{3}=['pdftk `ls ',obj.dirname,'/*.pdf` cat output ',obj.dirname,'/merge.pdf'];
    
    % convert pdf to swf
    cmd{4}=['pdf2swf -s framerate=30 ',obj.dirname,'/merge.pdf ',obj.filename,'.swf'];
    
    % set the framerate (the framerate command above doesn't seem
    % to work in pdf2swf 0.8.1)
    % also combine with swfstop.swf which is a flash file that is in
    % drake/util that sends the stop command to prevent the movie from
    % looping
    if (obj.loop)
      cmd{5}=['swfcombine -r ',num2str(obj.fps),' --dummy ',obj.filename,'.swf -o ',obj.filename, '.swf '];
    else
      cmd{5}=['swfcombine -r ',num2str(obj.fps),' --cat ',obj.filename,'.swf -o ',obj.filename, '.swf ', getDrakePath(), '/util/swfstop.swf'];
    end
    
    if (obj.poster)
      num_chars = 4;  % note error handling in addFrame
      
      % copy poster pdf next to swf output (useful for beamer)
      cmd{6}=['cp ',obj.dirname,'/',repmat('0',1,num_chars-1),'1.pdf ',obj.filename,'.pdf'];
    end
    
    for i=1:length(cmd)
      try
        if (systemWCMakeEnv([cmd{i}]) ~= 0) error('command failed'); end
      catch
        fprintf(1,'\n\n\nfailed running this on the command line:\n  ');
        disp(['  ',cmd{i}]);
        fprintf(1,'Make sure that you have the command line tool installed and have address all path/library issues.\n');
        error('swf writing failed');
      end
    end
    
    disp([obj.filename,' generated successfully']);
    
    rmdir(obj.dirname,'s');
    
    obj.dirname=[];
  end
  
  function obj = addFrame(obj,h)
    if (isempty(obj.dirname)) error('can''t addframe.  did you already close this swf?'); end
    if (nargin<2) h=gcf; end
    framenum = obj.frames+1;
    num_chars = 4;
    if (framenum>9999) 
      error('wow. need to handle this case.');  
      % eg., I could automatically rename all of the files with a simple
      % shell command here to add a leading zero.  
      % note that num_chars is also used in the close() method 
    end
    frame_fname=[obj.dirname,'/',repmat('0',1,num_chars-length(num2str(framenum))),num2str(framenum)];
    print(h,'-depsc',[frame_fname '.eps']);
%    export2pdf(frame_fname,h,struct('tight',true));
    obj.frames=framenum;
  end
  
end

end
