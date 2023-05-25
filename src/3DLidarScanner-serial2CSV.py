'''
#
# 
# * @file 3DLidarScanner-serial2CSV.py
# * @author Loïc Reboursière, Maxime Vander Goten, Sami Yunus - UMONS, CLick (loic.reboursiere@umons.ac.be, maxime.vandergoten@umons.ac.be, sami.yunus@umons.ac.be)
# * @brief This script communicates through serial with the 3DLidarScanner and can save to a csv file the cloudpoints delivered by the scanner
# * 3DLidarScanner-serial2CSV.py – CLICK - UMONS (Loïc Reboursière, Maxime Vander Goten, Sami Yunus) is free software: you can redistribute it and/or modify it under the terms of the Apache License Version 2.0. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Apache License Version 2.0 for more details.
# * You should have received a copy of the Apache License Version 2.0 along with this program.  If not, see http://www.apache.org/licenses/
# * Each use of this software must be attributed to University of MONS – CLICK (Loïc Reboursière, Maxime Vander Goten, Sami Yunus).
# * Any other additional authorizations may be asked to avre@umons.ac.be. 
# * @version 0.1
# * @date 2023-05-25
# * 
# * @copyright Copyright (c) 2023
# 
'''
 
# Connect the 3D Arduino-based LiDAR scanner performing the scan and launch this file.
# Use : 
#   python stepperScanner-serial2CSV.py --port the_port_of_connected_arduino_board --baudrate 115200
# 
# Once the serial communication is established, you can send any serial commands the scanner responds to :
#    - s : perform 2D scan
#    - y : perform 1D scan (vertical)
#    - i : display scanner setup info
#    - v : followed by an integer value sets the vertical step (e.g. `v 1`)
#    - h : followed by an integer value sets the horizontal step (e.g. `h 1`)
#    - p : scan reboot
#

import csv
from time import sleep

import serial
import argparse
import os, sys

def checkFileAndIncrement( folder_path, pattern ) :
    '''
    Count files of a specific folder with the same pattern.
    Arguments :
        folder_path : Path of the folder to investigate.
        pattern : Pattern to look for in files
    Return :
        Number of existing files. 0 means that none have been found.
    '''

    nb_of_existing_files = 0

    if folder_path == '' : folder_path = '.'

    for path in os.listdir( folder_path ) :
        if( pattern in path ):
            nb_of_existing_files += 1

    return nb_of_existing_files

def main (args) :

    filename = args.file
    ## TOCHECK : if it does what it's supposed to
    folder_path = os.path.dirname( filename )
    if folder_path == '' : folder_path = '.'    
    filename = os.path.join( folder_path, filename )
    
    if( os.path.isfile( filename ) ) :
        print( 'ERROR : Filename ' + filename + ' already exists, please enter a different filename.' )   
        sys.exit( 0 )
    else : 
        print( 'File doesn\'t exist')

    dataSerie = serial.Serial( port = args.port, baudrate = args.baudrate)
    print( '--- Connecting to port : ' + args.port + ' at ' + str( args.baudrate )  +' bauds' )

    listG = []
    nb_of_scan = 0

    ## Double while True loops need to be tested
    while True :
        data = input()
        #print( data )
        dataSerie.write( str.encode( data ) )
        
        if( data == 's' or data == 'y' ) :          

            while True:
                # sleep(0.1)

                mesure = dataSerie.readline().strip()
                mesureUtf = mesure.decode("utf-8")
                mesure1 = mesureUtf.split()

                print(mesure1)

                if mesureUtf == "end":

                    with open( filename, 'w' ) as f :
                        writer = csv.writer( f )
                        print( len( listG ) )
                        writer.writerows( listG )
                        break
                else: ## TODO : add all elements not just xYZ and then during parsing put them in the right dict keys
                    #listG.append( [ mesure1[ 6 ], mesure1[ 7 ], mesure1[ 8 ] + ";"] )
                    listG.append( mesureUtf )

        ## TOCHECK : do other serial commands send their feedback    
        else : ## you sent commands different from scanning commands
            data_raw = dataSerie.readline()
            if( data_raw == 'end' ) : 
                break 
            else : 
                print( data_raw )

        #else :
        #    while True:
        #        bytesToRead = dataSerie.inWaiting()
        #        print( dataSerie.read( bytesToRead ) )


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser( description = 'Receive serial data from scanner and record it into a csv file.' )
    parser.add_argument( "-p", "--port",
                        help = "Serial port used to connect Arduino Uno USB",
                        type = str, default = "" )
    parser.add_argument( "-b", "--baudrate", 
                        help = "Speed of the serial communication (default is 115200)",
                        type = int, default = "115200" )
    parser.add_argument( "-f", "--filename", 
                        help = "Filename of the stored data (default is scanning_data.csv)",
                        type = str, default = "sanning_data.csv" )
    args = parser.parse_args()

    main( args )