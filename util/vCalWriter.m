classdef vCalWriter
  
% there is a helpful ical spec listing here: http://www.kanzaki.com/docs/ical/

properties
    fhandle
    min_datenum
    max_datenum
  end
  
  methods
    function obj=vCalWriter(filename)
      if nargin<1
        [filename,pathname]=uiputfile('*.ics','Create ics file');
        filename=fullfile(pathname,filename);
      end
      typecheck(filename,'char');
      
      obj.fhandle=fopen(filename,'w');
      
      % write header
      fprintf(obj.fhandle,'BEGIN:VCALENDAR\r\n');
      fprintf(obj.fhandle,'VERSION:2.0\r\n');
      fprintf(obj.fhandle,'PRODID:Simple MATLAB vCalendar Script by russt@mit.edu\r\n');
      fprintf(obj.fhandle,'BEGIN:VTIMEZONE\r\n');
      fprintf(obj.fhandle,'TZID:US-Eastern\r\n');
      fprintf(obj.fhandle,'BEGIN:DAYLIGHT\r\n');
      fprintf(obj.fhandle,'DTSTART:20120801T000000Z\r\n'); % arbitrary date before start of class
      fprintf(obj.fhandle,'TZOFFSETTO:-040000\r\n');
      fprintf(obj.fhandle,'TZOFFSETFROM:-050000\r\n');
      fprintf(obj.fhandle,'END:DAYLIGHT\r\n');
      fprintf(obj.fhandle,'BEGIN:STANDARD\r\n');
      fprintf(obj.fhandle,'DTSTART:20121104T060000Z\r\n');
      fprintf(obj.fhandle,'TZOFFSETTO:-050000\r\n');
      fprintf(obj.fhandle,'TZOFFSETFROM:-040000\r\n');
      fprintf(obj.fhandle,'END:STANDARD\r\n');
      fprintf(obj.fhandle,'END:VTIMEZONE\r\n');
      
      obj.min_datenum = datenum([2012 08 01 00 00 00]);  % currently timezone implementation above is well-defined for these dates
      obj.max_datenum = datenum([2012 12 31 00 00 00]);
    end
    
    function addEvent(obj,summary,startdatevec,enddatevec,description,location,rrule)
      if datenum(startdatevec)<obj.min_datenum || datenum(startdatevec)>obj.max_datenum
        error('start date out of range.  you may need to update the timezone implementation in the vCalWriter class');
      end
      if nargin>3 && ~isempty(enddatevec) && (datenum(enddatevec)<obj.min_datenum || datenum(enddatevec)>obj.max_datenum)
        error('end date out of range.  you may need to update the timezone definition in the vCalWriter class');
      end
      fprintf(obj.fhandle,'BEGIN:VEVENT\r\n');

      typecheck(summary,'char');
      fprintf(obj.fhandle,'SUMMARY:%s\r\n',summary);
      
      switch length(startdatevec)
        case 3
          fprintf(obj.fhandle,'DTSTART;TZID=US-Eastern;VALUE=DATE:%02d%02d%02d\r\n',startdatevec);
        case 6
          fprintf(obj.fhandle,'DTSTART;TZID=US-Eastern:%02d%02d%02dT%02d%02d%02d\r\n',startdatevec);
        otherwise
          error('start date must be a 3 element or 6 element datevec');
      end
      if nargin>3 && ~isempty(enddatevec)
        sizecheck(enddatevec,6);
        fprintf(obj.fhandle,'DTEND;TZID=US-Eastern:%02d%02d%02dT%02d%02d%02d\r\n',enddatevec);
      end

      if nargin>4 && ~isempty(description)
        typecheck(description,'char');
        fprintf(obj.fhandle,'LOCATION:%s\r\n',description);
      end
      
      if nargin>5 && ~isempty(location)
        typecheck(location,'char');
        fprintf(obj.fhandle,'LOCATION:%s\r\n',location);
      end
      
      if nargin>6 && ~isempty(rrule)
        typecheck(rrule,'char');
        fprintf(obj.fhandle,'RRULE:%s\r\n',rrule);
      end
      
      fprintf(obj.fhandle,'END:VEVENT\r\n');
    end
    
    function delete(obj)
      fprintf(obj.fhandle,'END:VCALENDAR\r\n');
      fclose(obj.fhandle);
    end
  end
end