'''
# Connect the 3D Arduino-based LiDAR scanner performing the scan and launch this file.
# Use : 
#   python stepperScanner-serial2CSV.py --port the_port_of_connected_arduino_board --baudrate 115200
# 
# Serial command :
#    s : make a 3D scan
#    y : make a 2D scan
'''
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

    print( '--- Connecting to port : ' + args.port + ' at ' + str( args.baudrate )  +' bauds' )
    filename = args.file
    dataSerie = serial.Serial( port = args.port, baudrate = args.baudrate)

    listG = []
    nb_of_scan = 0

    ## Double while True loops need to be tested
    while True :
        data = input()
        #print( data )
        dataSerie.write(str.encode(data))
        
        if( data == 's' or data == 'y' ) :
            nb_of_scan += 1
            dimension = ''
            suffix = ''

            if( data == 's' ):
                dimension = '3D'
            elif( data =='y' ):
                dimension == '2D'
            
            ## TOCHECK : if it does what it's supposed to
            folder_path = os.path.dirname( filename )
            if folder_path == '' : folder_path = '.'    
            split = os.path.basename( filename ).split( '.' )
            filename = os.path.join( folder_path, split[ 0 ] + "-" + dimension + ".csv" )
            
            if( os.path.isfile( filename ) ) :
                print( 'Filename ' + filename + 'already exists, please enter a different filename.' )   
                sys.exit( 1 )
                

            while True:
                # sleep(0.1)

                mesure = dataSerie.readline().strip()
                mesureUtf = mesure.decode("utf-8")
                mesure1 = mesureUtf.split()

                print(mesure1)

                if mesureUtf == "stop":

                    with open( filename, 'w' ) as f :
                        writer = csv.writer( f )
                        print( len( listG ) )
                        writer.writerows( listG )
                        break
                else: ## TODO : add all elements not just xYZ and then during parsing put them in the right dict keys
                    #listG.append( [ mesure1[ 6 ], mesure1[ 7 ], mesure1[ 8 ] + ";"] )
                    listG.append( mesureUtf )
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