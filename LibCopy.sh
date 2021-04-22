#!/bin/bash

echo ">>>>=====Robot Libs Copy Script=====<<<<"

echo "> Usage:"
echo "> LibCopy.sh [-l] [-m] [-c]"
echo "> Description:"
echo "> -l copy all libs under the dir RobotLibs."
echo "> -m copy executable RobotMain."
echo "> -c copy libRobotControlLib.so under the dir RobotControlLib."


while getopts 'lmc' OPT; do
    case $OPT in
        l)
        echo "Copy RobobLibs/*.so* to destination."
        COPY_LIBS='true'

        ;;
        m)
        echo "Copy RobotMain to destination."
        COPY_ROBOTMAIN='true'
        ;;
        c)
        echo "Copy libRobotControlLib.so to destination."
        COPY_CONTROL='true'
        ;;
        *)
        echo "Undefined Options $OPTARG"
        ;;
    esac
done





