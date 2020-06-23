#!/bin/bash
function usage() {
    echo "$0 script"
    exit 1

}
if [ $# -lt 1 ];then
    usage
else
    script_name=$1
    matlab -nodisplay -nosplash -nodesktop -r "run('${script_name}');exit;"
    #matlab -nosplash -nodesktop -r "run('${script_name}');exit;"
fi
