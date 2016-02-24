#!/bin/bash

# 18649-Fall-2015
# Group 3
# Jiyu Shi(jiyus) ; Shuai Wang(shuaiwa1); Xiaoyu Wang(xiaoyuw); Xiao Guo(xiaog)

# This program generates passenger files for acceptance testing the elevator.
#
# Passengers enter on random floor and random hallway, at random time. Floors 
# and hallways are valid.
#
# Subsequent lines are passenger injections and have the following format:
#
#           Time	Start Floor	Start Hallway	End Floor	End Hallway 

MIN_ARG_NUM=3;

if [ $# -lt $MIN_ARG_NUM ] 
    then 
        #Replace these with actual usage directions
        echo "<USAGE INFO>";
          echo "   <sh AcceptTestGenerator.sh passNum outFile [optional] situation>";
          echo "Parameters:";
          echo " passNum : The total number of passengers in the test";
          echo " outFile : Output the test data to a .pass file";

          echo " situation : Use \"uppeak\" to generate up peak scenarios. Use \"downpeak\" to generate down peak scenarios. Otherwise, it is normal case.";

        echo "<EXAMPLE COMMAND LINE CALL>";
          echo "  sh AcceptTestGenerator.sh 100 test1.pass uppeak ";

else
        PASS_NUM=$1; #Read the arguments into local variables
        OUTFILE=$2;
        BIAS=$3;        
        echo ";Random Generated Pass file. Number of Passengers = $PASS_NUM. Situation = $BIAS." > $OUTFILE;
        u=0;
        d=0;

        if [ $BIAS = "uppeak" ]  
            then           
              u=1;
        
        fi

        if [ $BIAS = "downpeak" ]
           then                       
              d=1;
        fi
        
        t=0;
        for ((i = 1; i <= $PASS_NUM; i++));
            do
                dt=$(($RANDOM % 20 + 1 ));
                t=$((t+dt));
                s="$((t))s "; #Passenger insertion time

                floor=$(($RANDOM % 8 + 1 )); #Passenger starts floor
                r=$(($RANDOM % 10 + 1 )); #deal with up peak
                floor=$((u==1 && r>1 ? 1 : floor));

                s="$s $floor ";  
                case $floor in 
                    1 )
                        hallway=$(($RANDOM % 2 ));
                        case $hallway in
                            0 )
                                s="$s FRONT ";
                                ;;
                            1 )
                                s="$s BACK ";
                                ;;
                        esac 
                        ;;
                    2 )
                        s="$s BACK ";
                        ;;
                    3 )
                        s="$s FRONT ";
                        ;;
                    4 )
                        s="$s FRONT ";
                        ;;
                    5 )
                        s="$s FRONT ";
                        ;;
                    6 )
                        s="$s FRONT ";
                        ;;
                    7 )
                        hallway=$(($RANDOM % 2 ));
                        case $hallway in
                            0 )
                                s="$s FRONT ";
                                ;;
                            1 )
                                s="$s BACK ";
                                ;;
                        esac 
                        ;;
                    8 )
                        s="$s FRONT "
                        ;;                      
                          
                esac

                floor=$(($RANDOM % 8 + 1 )); # passenger end floor
                r=$(($RANDOM % 10 + 1 )) #deal with down peak
                floor=$((d==1 && r>1? 1:floor));
                s="$s $floor "; # and ends floor
                case $floor in 
                    1 )
                        hallway=$(($RANDOM % 2 ));
                        case $hallway in
                            0 )
                                s="$s FRONT ";
                                ;;
                            1 )
                                s="$s BACK ";
                                ;;
                        esac 
                        ;;
                    2 )
                        s="$s BACK ";
                        ;;
                    3 )
                        s="$s FRONT ";
                        ;;
                    4 )
                        s="$s FRONT ";
                        ;;
                    5 )
                        s="$s FRONT ";
                        ;;
                    6 )
                        s="$s FRONT ";
                        ;;
                    7 )
                        hallway=$(($RANDOM % 2 ));
                        case $hallway in
                            0 )
                                s="$s FRONT ";
                                ;;
                            1 )
                                s="$s BACK ";
                                ;;
                        esac 
                        ;;
                    8 )
                        s="$s FRONT "
                        ;;                      
                          
                esac

                echo "$s" >> $OUTFILE
            done
                
fi

