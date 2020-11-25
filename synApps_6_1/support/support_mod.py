import os
import sys

support_dirname = os.getcwd();
print "Support directory is " + support_dirname;

template_filename = "configure/RELEASE.tmpl{}".format(sys.argv[1]);
release_filename = "configure/RELEASE";

template_file = open(template_filename, "rb");
release_file = open(release_filename, "wb");

for curr_line in template_file:
    if (curr_line.startswith("SUPPORT=%SUPPORT")):
        curr_line = "SUPPORT="+support_dirname+"\n";

    release_file.write(curr_line);

template_file.close();
release_file.close();


