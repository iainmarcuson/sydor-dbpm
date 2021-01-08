import sys

out_filename = 'iocBoot/iocBS_EM/calibration.ini';
in_filename = 'dbpm_calibration.ini';
calibration_name = sys.argv[1];

STATE_COPY=1;
STATE_IGNORE=0;

out_file = open(out_filename, 'w');
in_file = open(in_filename, 'r');

cal_state = STATE_IGNORE;       # Start out not copying lines
find_string = '[{}_range'.format(calibration_name); # Get pattern
for curr_line in in_file:
    strip_line = curr_line.strip();
    out_line = '\n';            # Start with a benign line
    if cal_state == STATE_IGNORE:
        if strip_line.startswith(find_string):
            cal_state = STATE_COPY; # Note that we will copy lines now
            split_line = strip_line.split('_',1); # Separate name from param
            split_line[0] = '[direct';            # Rewrite with used name
            out_line ='_'.join(split_line);      # Reassemble
            out_line = out_line+'\n';             # Append newline
    else:                                     # Copying a line
        if strip_line.startswith('[') and (not strip_line.startswith(find_string)):
            cal_state = STATE_IGNORE;
        elif strip_line.startswith(find_string):
            split_line = strip_line.split('_',1); # Separate name from param
            split_line[0] = '[direct';
            out_line = '_'.join(split_line);
            out_line = out_line+'\n';
        else:
            out_line = curr_line;

    if cal_state == STATE_COPY:
        out_file.write(out_line);

out_file.close();
in_file.close();

