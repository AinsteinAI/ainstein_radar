#!/usr/bin/env/python

import sys
import errno
import struct
import math

if sys.version_info[0] == 3:
    import tkinter as tk # python 3
    from tkinter import filedialog as filedialog
else:
    import Tkinter as tk # python 2
    import tkFileDialog as filedialog
    
import can
import time
from collections import OrderedDict

RADAR_SYSTEM_CMD = 0x100
RADAR_SYSTEM_RES = 0x101

RADAR_START = 0x01
RADAR_STOP = 0x02

RADAR_SEND_TRACKED = 0x01
RADAR_SEND_RAW = 0x02
RADAR_SEND_BOTH = 0x03

RADAR_SET_PARAM = 0x03

RADAR_RESERVED = 0xff

ENTRY_WIDTH = 15
ENTRY_COLSPAN = 2

class Window( tk.Frame ):
    def __init__( self, master=None ):
        tk.Frame.__init__( self, master )
        self.master = master

        self.is_radar_connected = False

        self.bus = None
        
        self.params = OrderedDict()
        self.params["can_interface"] = \
            ( "can_interface", "CAN Interface",
              ["can0", "can1", "can2", "can3"],
              "can0",
              4*[None]
            )
        self.params["can_id"] = \
            ( "can_id", "CAN ID",
              map( str, range( 0, 10 ) ),
              "0",
              [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09]
            )
        self.params["can_baudrate"] = \
            ( "can_baudrate", "CAN Baudrate",
              ["1 Mbps", "500 Kbps", "250 Kbps"],
              "500 Kbps",
              [0x02, 0x03, 0x04]
            )
        self.params["enable_self_start"] = \
            ( "enable_self_start", "Enable Self Start",
              ["True", "False"],
              "False",
              [0x00, 0x01]
            )
                        
        self.param_labels = {}
        self.param_entry = {}
        self.param_var = {}
        
        self.init_window()

        
    def init_window( self ):
        self.master.title( "T79 Configuration App" )
        self.grid()

        # Set up the Entry widgets for each network parameter:
        for ind, (key, param) in enumerate( self.params.items() ):
            self.param_labels[param[0]] = tk.Label( self, text=param[1] )
            self.param_labels[param[0]].grid( row=ind, column=0 )

            self.param_var[param[0]] = tk.StringVar(self)
            self.param_var[param[0]].set(param[2][param[2].index(param[3])])
            self.param_entry[param[0]] = tk.OptionMenu( self, self.param_var[param[0]], *param[2] )
            self.param_entry[param[0]].grid( row=ind, column=1, columnspan=ENTRY_COLSPAN )
            self.param_entry[param[0]].config( state=tk.DISABLED )

        # Set the Radar CAN Interface entry widget to ENABLED and give it focus:
        self.param_entry["can_interface"].config( state=tk.NORMAL, fg="black" )
        self.param_entry["can_interface"].focus_set()
            
        # Connect Button:
        self.connect_button = tk.Button( self, text="CONNECT", command=self.connect_to_radar )
        self.connect_button.grid( row=4, column=0 )
        
        # Send Configuration Button:
        self.send_config_button = tk.Button( self, text="CONFIGURE", command=self.send_config_to_radar )
        self.send_config_button.grid( row=4, column=1 )
        self.send_config_button.config( state=tk.DISABLED )
        
        # Start Radar Button:
        self.start_radar_button = tk.Button( self, text="START", command=self.start_radar )
        self.start_radar_button.grid( row=4, column=2 )
        self.start_radar_button.config( state=tk.DISABLED )
        
        # Run Radar Button:
        self.stop_radar_button = tk.Button( self, text="STOP", command=self.stop_radar )
        self.stop_radar_button.grid( row=4, column=3 )
        self.stop_radar_button.config( state=tk.DISABLED )

        # Display for output:
        self.text_display = tk.Text( self, width=65, height=10 )
        self.text_display.grid( row=0, column=4, rowspan=5 )
        self.update_text_display_newline( "Please enter the radar socketCAN interface name (for example, can0) and press CONNECT." )

        # Scrollbar for display text:
        self.scrollbar_display = tk.Scrollbar( self, command=self.text_display.yview )
        self.scrollbar_display.grid( row=0, column=5, rowspan=5 )
        self.text_display['yscrollcommand'] = self.scrollbar_display.set

        
    def connect_to_radar( self ):
        if not self.is_radar_connected:
            self.bus = can.Bus( interface='socketcan',
                                channel=self.param_var["can_interface"].get(),
                                receive_own_messages=False )
            self.update_text_display_newline( ">> Connected to CAN interface " + self.param_var["can_interface"].get() )
        else:
            self.update_text_display_newline( ">> Already connected." )
        
        # Enable the entry widgets and buttons:
        [val.config( state=tk.NORMAL, fg="black" ) for key, val in self.param_entry.items()]
        self.send_config_button.config( state=tk.NORMAL )
        self.start_radar_button.config( state=tk.NORMAL )
        self.stop_radar_button.config( state=tk.NORMAL )

        self.update_text_display_newline( "Ensure the radar is STOPPED, enter the desired radar parameters and press CONFIGURE to send." )

        self.is_radar_connected = True

        
    def send_config_to_radar( self ):
        if not self.is_radar_connected:
            self.update_text_display_newline( ">> You must connect to radar before sending parameters." )
            return

        # Parse out the parameters:
        can_id = self.params["can_id"][4][self.params["can_id"][2].index(self.param_var["can_id"].get())]
        can_baudrate = self.params["can_baudrate"][4][self.params["can_baudrate"][2].index(self.param_var["can_baudrate"].get())]
        enable_self_start = self.params["enable_self_start"][4][self.params["enable_self_start"][2].index(self.param_var["enable_self_start"].get())]
        
        
        # Create the parameter configuration command:
        config_msg = can.Message( arbitration_id=RADAR_SYSTEM_CMD,
                                  data=[RADAR_SET_PARAM,
                                        can_id,
                                        RADAR_RESERVED,
                                        enable_self_start,
                                        RADAR_RESERVED,
                                        can_baudrate,
                                        RADAR_RESERVED,
                                        RADAR_RESERVED],
                                  is_extended_id=False )

        # Send the parameter configuration command:
        try:
            self.bus.send( config_msg )
            
        except can.CanError:
            self.update_text_display_newline( ">> Failed to send configuration." )

        self.update_text_display_newline( ">> Configuration sent." )
        self.update_text_display_newline( "START the radar and candump data to verify changes." )
            
    def start_radar( self ):
        # Create start command, enabling both raw and tracked targets by default:
        msg = can.Message( arbitration_id=RADAR_SYSTEM_CMD,
                           data=[RADAR_START,
                                 RADAR_RESERVED,
                                 RADAR_SEND_BOTH,
                                 RADAR_RESERVED,
                                 RADAR_RESERVED,
                                 RADAR_RESERVED,
                                 RADAR_RESERVED,
                                 RADAR_RESERVED],
                           is_extended_id=False )        

        # Send the start command:
        try:
            self.bus.send( msg )
            
        except can.CanError:
            self.update_text_display_newline( ">> Failed to send start command." )

        self.update_text_display_newline( ">> Radar started." )
        self.update_text_display_newline( "Monitor data with pcanview or candump. Be sure to STOP the radar again before reconfiguring." )

            
    def stop_radar( self ):
        # Create stop command, enabling both raw and tracked targets by default:
        msg = can.Message( arbitration_id=RADAR_SYSTEM_CMD,
                           data=[RADAR_STOP,
                                 RADAR_RESERVED,
                                 RADAR_SEND_BOTH,
                                 RADAR_RESERVED,
                                 RADAR_RESERVED,
                                 RADAR_RESERVED,
                                 RADAR_RESERVED,
                                 RADAR_RESERVED],
                           is_extended_id=False )

        # Send the stop command:
        try:
            self.bus.send( msg )
            
        except can.CanError:
            self.update_text_display_newline( ">> Failed to send stop command." )

        self.update_text_display_newline( ">> Radar stopped." )

            
    def update_text_display_newline( self, newtext ):
        self.text_display.config( state=tk.NORMAL )
        self.text_display.insert( tk.END, newtext +"\n" )
        self.text_display.config( state=tk.DISABLED )
        self.text_display.see( "end" )

        
    def update_text_display( self, newtext ):
        self.text_display.config( state=tk.NORMAL )
        self.text_display.insert( tk.END, newtext )
        self.text_display.config( state=tk.DISABLED )
        self.text_display.see( "end" )

        
    def overwrite_text_display( self, newtext ):
        self.text_display.config( state=tk.NORMAL )
        self.text_display.delete( 'end-1l', 'end' )
        self.text_display.insert( tk.END, newtext )
        self.text_display.config( state=tk.DISABLED )
        self.text_display.see( "end" )

        
if __name__ == "__main__":
    root = tk.Tk()
    root.geometry( "900x175" )
    root.resizable( 0, 0 )
    app = Window( root )
    root.mainloop()

    
