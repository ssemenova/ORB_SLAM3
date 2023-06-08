#!/bin/bash
### Script Usage ###
show_usage() {
    echo "This script extracts CPU and memory usage from top command for a specific process and redirect it to a sperate file."
    echo -e "\nUsage:\n./top-memcpu.sh <iterations> <process-name> <output-filename>\n"
}

# check if number of arguments supplied is correct. If not, then display usage
if [ $# -le 1 ]
then
    show_usage
    exit 1
fi

# check whether user had supplied -h or --help. If yes, then display usage
for i in $@
do
    if [[ ( $i == "--help" ) || ( $i == "-h" ) ]]
    then
        show_usage
        exit 0
    fi
done

# display usage if the script is run as root user
if [[ $USER == "root" ]]
then
    echo "Error: this script should NOT run as root!"
    exit 1
fi
##########

echo "===== start ====="

# Delete output file if it exists
[ -e $2 ] && rm -v -- $3

while [ true ]
do
    # Output
    echo "collect-mem-cpu count: $i"

    # Measurements
    # Timestamp in milliseconds
    timestamp="$(date +%s%N | cut -b1-13)"
    echo "time,$timestamp" >> $2
    COLUMNS=512 top -c -b -n 1 | grep -Ei "[%]CPU|[%]MEM|[K]iB|[R]GBD .*config" >> $2

    # Sleep one second
    sleep 1s
done

echo "===== done ====="

