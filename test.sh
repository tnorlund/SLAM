#!/bin/bash
for time in 15 20 25 30 35 40 45 50 55 60 
do
    for fps in 10 15 20 25 30
    do
        for buffer in 100 200 300 400 500 600 700 800
        do 
            ./frameratetest -v --time $time --fps $fps --buffer $buffer --tablename $TABLENAME &
            export JOB=`jobs -p`
            wait $JOB
        done
    done
done