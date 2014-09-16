function export2pdf(filename,h,options)
% exports current figure to filename.pdf
% @param filename should have no extension... pdf will be added
% @param h optional handle.  default is gcf
%
% @option tight set to true to try to get a small bounding box @default
% true

if (nargin<2 || isempty(h)) 
  h = get(0,'CurrentFigure'); 
  if (isempty(h)) error('no figure to export'); end
end
if (nargin<3) options=struct(); end
if (~isfield(options,'tight')) options.tight=true; end
if (~isfield(options,'bbox')) options.bbox=[]; end

if (options.tight || ~isempty(options.bbox))
  
  % then use epsc and epstopdf
  print(h,'-depsc',[filename '.eps']);
  if (~isempty(options.bbox))
    system(['sed -e "s/BoundingBox:.*$/BoundingBox: ',num2str(options.bbox),'/g" -i "" ', filename,'.eps']);
  end
  if (systemWCMakeEnv(['epstopdf --outfile=', filename, '.pdf ', filename, '.eps']) ~= 0) error('epstopdf failed.'); end
  delete([filename,'.eps']);
  
else
  
  % Backup previous settings
  prePaperType = get(h,'PaperType');
  prePaperUnits = get(h,'PaperUnits');
  preUnits = get(h,'Units');
  prePaperPosition = get(h,'PaperPosition');
  prePaperSize = get(h,'PaperSize');

  % Make changing paper type possible
  set(h,'PaperType','<custom>');

  % Set units to all be the same
  set(h,'PaperUnits','inches');
  set(h,'Units','inches');

  % Set the page size and position to match the figure's onscreen
  % dimensions
  position = get(h,'Position');
  set(h,'PaperPosition',[0,0,position(3:4)]);
  set(h,'PaperSize',position(3:4));

  % Save the pdf
  % we do this instead of going to eps so that we'll print onto a
  % page, which keeps our size the same between frames that have
  % different sizes.
  print(h,'-dpdf',[filename '.pdf']);

  % Restore the previous settings
  set(h,'PaperType',prePaperType);
  set(h,'PaperUnits',prePaperUnits);
  set(h,'Units',preUnits);
  set(h,'PaperPosition',prePaperPosition);
  set(h,'PaperSize',prePaperSize);

end


