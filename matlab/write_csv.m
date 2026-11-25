function write_csv(name, header, data)
% WRITE_CSV  Write a matrix with a header line to ../typst/data/control/<name>.csv.
dir = fullfile(fileparts(mfilename('fullpath')), '..', 'typst', 'data', 'control');
fid = fopen(fullfile(dir, [name '.csv']), 'w');
fprintf(fid, '%s\n', header);
fmt = [strjoin(repmat({'%.9g'}, 1, size(data, 2)), ','), '\n'];
fprintf(fid, fmt, data');
fclose(fid);
end
