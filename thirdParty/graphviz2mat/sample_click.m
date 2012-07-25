text_elms = findall(gca,'Type','text');  
for ndx = 1:length(text_elms)
    callbk = 'my_call(str2num(get(gcbo,''String'')))'; 
    set(text_elms(ndx), 'ButtonDownFcn', callbk);  % assume the node label is a number
end

function varargout = my_call(value)
label = get(gcbo,'String');  % "gcbo" is the handle of the object whose callback is executing
fprintf('You clicked: %s \t', label)  