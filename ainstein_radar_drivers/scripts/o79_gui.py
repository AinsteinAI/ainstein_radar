#!/usr/bin/env/python

import sys
import errno
import struct
import math

if sys.version_info[0] == 3:
    import tkinter as tk # python 3
    from tkinter import filedialog as filedialog
    from tkinter import ttk
    from tkinter import messagebox
else:
    import Tkinter as tk # python 2
    import tkFileDialog as filedialog
    import ttk
    import tkMessageBox as messagebox
    
import socket
import time

RADAR_PORT = 7

CONNECT_CMD = "connect"
CONNECT_RES = "Connection established."

PRIMARY_UPDATE_CMD = "update"  # this needs to be "update" to be compatible with initial fw released to BC
GOLDEN_UPDATE_CMD = "golden"
UPDATE_RES = "Ready IP config"
UPDATE_DONE_CMD = "done"
CHUNK_SIZE = 256 # bytes

CONFIG_CMD = "config"
CONFIG_RES = "Done."
CONFIG_COMPLETE_RES = "All done."

RUN_CMD = "run"

PARAMS_CMD = "params"

ENTRY_WIDTH = 15
ENTRY_COLSPAN = 2

NO_VERSION = '0.0.0'

class Window( tk.Frame ):
    def __init__( self, master=None ):
        tk.Frame.__init__( self, master )
        self.master = master

        self.tab_ctrl = ttk.Notebook(self.master)
        self.tab1 = ttk.Frame(self.tab_ctrl)
        self.tab2 = ttk.Frame(self.tab_ctrl)

        self.tab_ctrl.add(self.tab1, text="Network")
        self.tab_ctrl.add(self.tab2, text="Tracking Filter")
        self.tab_ctrl.pack(expand=1, fill="both")

        self.style_entry_disabled = ttk.Style()
        self.style_entry_disabled.configure( "disabled.TEntry", foreground="grey" )
        self.style_entry_enabled = ttk.Style()
        self.style_entry_enabled.configure( "enabled.TEntry", foreground="black" )
        
        self.current_radar_ip = ""
        self.is_radar_connected = False
        self.fwVersion = NO_VERSION

        self.sock = None
        
        self.network_params = ( ( "radar_ip", "Radar IP", "10.0.0.10" ),
                                ( "radar_netmask", "Radar Netmask", "255.255.255.0" ),
                                ( "radar_gateway", "Radar Gateway", "0.0.0.0" ),
                                ( "host_ip", "Host PC IP", "10.0.0.75" ),
                                ( "host_port", "Host PC Port", "1024" ) )
        self.network_param_labels = {}
        self.network_param_entry = {}

        self.filter_params = ( ( "filter_process_rate", "Filter Process Rate (Hz)", "20.0" ),
                               ( "filter_min_time", "Filter Minimum Time (s)", "2.0" ),
                               ( "filter_timeout", "Filter Timeout (s)", "0.5" ),
                               ( "filter_confidence_level", "Filter Confidence Level", "3" ),
                               ( "filter_process_noise_speed", "Filter Speed Process Noise (m/s)", "0.1" ),
                               ( "filter_process_noise_azim", "Filter Azimuth Process Noise (deg)", "5.0" ),
                               ( "filter_process_noise_elev", "Filter Elevation Process Noise (deg)", "5.0" ),
                               ( "filter_update_min_range", "Filter Update Minimum Range (m)", "0.2" ),
                               ( "filter_update_max_range", "Filter Update Maximum Range (m)", "15.0" ) )
        self.filter_param_labels = {}
        self.filter_param_entry = {}

        # Handle window close event gracefully:
        self.master.protocol( "WM_DELETE_WINDOW", self.close_window )
        
        self.init_window()

        
    def close_window( self ):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            # Send FIN and deallocate open TCP socket:
            if self.sock is not None and self.is_radar_connected:
                self.sock.shutdown(1)
                self.sock.close()

            # Destroy the main window (root):
            self.master.destroy()

            
    def set_config_state_all_enabled( self, enabled=True ):
        if enabled is True:
            config_state = tk.NORMAL
        elif enabled is False:
            config_state = tk.DISABLED
        else:
            print( "ERROR >> Invalid option." )
            return
        
        # Disable network parameter entries
        for ind, param in enumerate( self.network_params ):
            self.network_param_entry[param[0]].config( state=config_state )

        # Disable filter parameter entries
        for ind, param in enumerate( self.filter_params ):
            self.filter_param_entry[param[0]].config( state=config_state )

        # Disable buttons
        self.connect_button.config( state=config_state )
        self.send_config_button.config( state=config_state )
        self.update_firmware_button.config( state=config_state )
        self.update_golden_button.config( state=config_state )
        self.run_radar_button.config( state=config_state )
        self.send_params_button.config( state=config_state )

            
    def init_window( self ):
        self.master.title( "O-79 Network App" )

        # Set up the Entry widgets for each network parameter:
        for ind, param in enumerate( self.network_params ):
            self.network_param_labels[param[0]] = ttk.Label( self.tab1, text=param[1] )
            self.network_param_labels[param[0]].grid( row=ind, column=0 )
            
            self.network_param_entry[param[0]] = ttk.Entry( self.tab1, width=ENTRY_WIDTH, style="disabled.TEntry" )
            self.network_param_entry[param[0]].grid( row=ind, column=1, columnspan=ENTRY_COLSPAN )
            self.network_param_entry[param[0]].delete( 0, tk.END )
            self.network_param_entry[param[0]].insert( 0, param[2] )
            self.network_param_entry[param[0]].config( state=tk.DISABLED )

        # Set the Radar IP entry widget to ENABLED and give it focus:
        self.network_param_entry["radar_ip"].config( state=tk.NORMAL, style="enabled.TEntry" )
        self.network_param_entry["radar_ip"].focus_set()

        # Set the Host IP entry widget to ENABLED and give it focus:
        self.network_param_entry["host_ip"].config( state=tk.DISABLED, style="enabled.TEntry" )
            
        # Connect Button:
        self.connect_button = ttk.Button( self.tab1, text="Connect", command=self.connect_to_radar )
        self.connect_button.grid( row=5, column=0 )

        # Send Configuration Button:
        self.send_config_button = ttk.Button( self.tab1, text="Configure", command=self.send_config_to_radar )
        self.send_config_button.grid( row=5, column=1 )
        self.send_config_button.config( state=tk.DISABLED )

        # Update Firmware Button:
        self.update_firmware_button = ttk.Button(self.tab1, text="Update",
                                                command = lambda: self.update_firmware(PRIMARY_UPDATE_CMD))
        self.update_firmware_button.grid( row=6, column=0 )
        self.update_firmware_button.config( state=tk.DISABLED )

        # Update Golden Image Button:
        self.update_golden_button = ttk.Button(self.tab1, text="Update Golden Image",
                                              command = lambda: self.update_firmware(GOLDEN_UPDATE_CMD))
        self.update_golden_button.grid( row=6, column=1 )
        self.update_golden_button.config( state=tk.DISABLED )

        # Run Radar Button:
        self.run_radar_button = ttk.Button( self.tab1, text="Run", command=self.run_radar )
        self.run_radar_button.grid( row=5, column=2 )
        self.run_radar_button.config( state=tk.DISABLED )

        # Display for output:
        self.text_display = tk.Text( self.tab1, width=65, height=10 )
        self.text_display.grid( row=0, column=5, rowspan=8 )
        self.update_text_display_newline( "Please enter the IP address of your radar, then connect." )

        # Scrollbar for display text:
        self.scrollbar_display = ttk.Scrollbar( self.tab1, command=self.text_display.yview )
        self.scrollbar_display.grid( row=0, column=6, rowspan=8 )
        self.text_display['yscrollcommand'] = self.scrollbar_display.set

        # Set up the Entry widgets for each filter parameter:
        for ind, param in enumerate( self.filter_params ):
            self.filter_param_labels[param[0]] = ttk.Label( self.tab2, text=param[1] )
            self.filter_param_labels[param[0]].grid( row=ind, column=0 )
            
            self.filter_param_entry[param[0]] = ttk.Entry( self.tab2, width=ENTRY_WIDTH, style="disabled.TEntry" )
            self.filter_param_entry[param[0]].grid( row=ind, column=1, columnspan=ENTRY_COLSPAN )
            self.filter_param_entry[param[0]].delete( 0, tk.END )
            self.filter_param_entry[param[0]].insert( 0, param[2] )
            self.filter_param_entry[param[0]].config( state=tk.DISABLED )

        # Send Parameters Button:
        self.send_params_button = ttk.Button( self.tab2, text="Send Parameters", command=self.send_params_to_radar )
        self.send_params_button.grid( row=9, column=0 )
        self.send_params_button.config( state=tk.DISABLED )

    def connect_to_radar( self ):
        if not self.is_radar_connected:
            # Create a new socket instance:
            self.sock = socket.socket( socket.AF_INET, # Internet
                                       socket.SOCK_STREAM ) # TCP
            #self.sock.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )

            # Set the socket timeout:
            self.sock.settimeout( 3 )

            # Connect the socket to the server (radar):
            try:
                self.sock.connect( ( self.network_param_entry["radar_ip"].get(), 7 ) ) # radar port is always 7 (ECHO port)
            except socket.error as err:
                if err.errno == errno.EADDRNOTAVAIL:
                    self.update_text_display_newline( "ERROR >> Address not available. Is " + self.network_param_entry["radar_ip"].get() + " the IP of your radar?" )
                else:
                    self.update_text_display_newline( "ERROR >> " + str( err ) )
                return

            # Send the connect command and receive the stored network configuraton in response:
            self.current_radar_ip = self.network_param_entry["radar_ip"].get()

            self.sock.sendall( CONNECT_CMD )

            try:
                msg = self.sock.recv( 1024 )
                self.update_text_display_newline( "Loading network parameters stored on radar... Done." )
                time.sleep(1)

                # Enable the entry widgets and buttons:
                [val.config( state=tk.NORMAL, style="enabled.TEntry" ) for key, val in self.network_param_entry.items()]
                [val.config( state=tk.NORMAL, style="enabled.TEntry" ) for key, val in self.filter_param_entry.items()]
                self.update_firmware_button.config( state=tk.NORMAL )
                self.update_golden_button.config( state=tk.NORMAL )
                self.send_config_button.config( state=tk.NORMAL )
                self.run_radar_button.config( state=tk.NORMAL )
                self.send_params_button.config( state=tk.NORMAL )

                # Update the Entry widgets based on radar stored params:
                self.network_param_entry["radar_ip"].delete( 0, tk.END )
                self.network_param_entry["radar_ip"].insert( 0, '.'.join( map( str, struct.unpack_from( "<4I", msg, offset=0 ) ) ) )


                self.network_param_entry["radar_netmask"].delete( 0, tk.END )
                self.network_param_entry["radar_netmask"].insert( 0, '.'.join( map( str, struct.unpack_from( "<4I", msg, offset=16 ) ) ) )

                self.network_param_entry["radar_gateway"].delete( 0, tk.END )
                self.network_param_entry["radar_gateway"].insert( 0, '.'.join( map( str, struct.unpack_from( "<4I", msg, offset=32 ) ) ) )

                self.network_param_entry["host_ip"].delete( 0, tk.END )
                self.network_param_entry["host_ip"].insert( 0, '.'.join( map( str, struct.unpack_from( "<4I", msg, offset=48 ) ) ) )

                self.network_param_entry["host_port"].delete( 0, tk.END )
                self.network_param_entry["host_port"].insert( 0, '.'.join( map( str, struct.unpack_from( "<I", msg, offset=64 ) ) ) )
                try:
                    self.fwVersion = '.'.join( map( str, struct.unpack_from( "<3B", msg, offset=68 ) ))
                except:
                    self.fwVersion = NO_VERSION
                self.is_radar_connected = True
                self.update_text_display_newline( "Current radar firmware version is " + self.fwVersion)
                self.update_text_display_newline( "Change any network parameters, then press Configure to send." )

            except socket.error as err:
                self.update_text_display_newline( "ERROR >> Failed to connect to radar." )
                return
            except socket.timeout as err:
                self.update_text_display_newline( "ERROR >> Connect timed out. Is the radar connected and powered?" )
                return

            
    def update_firmware( self, cmd ):
        if not self.is_radar_connected:
            self.update_text_display_newline( "ERROR >> Must connect to radar before updating firmware." )
            return

        # Read in firmware file path from user input:
        bin_path = filedialog.askopenfilename(filetypes = [('Binary Files', '*.bin')])
        self.set_config_state_all_enabled( False ) # disable all entries and buttons

        # Send start configuration command:
        try:
            self.sock.sendall( cmd )
        except socket.error as err:
            self.update_text_display_newline( str(err) )
            return

        msg = self.sock.recv( 1024 )
        self.update_text_display_newline( self.current_radar_ip + " sent: " + msg )

        self.master.update()
        time.sleep( 3 )

        self.update_text_display( "Sending file..." )

        # Open the file and sent it in CHUNK_SIZE byte chunks:
        with open( bin_path, 'rb' ) as fd:
            # Read the file into a string (char array):
            firmware_arr = fd.read()

            # First, send the header with a corrupt value instead of XNLX. This way if the update
            # is interrupted, the bootloader will fall back to the golden image
            fc = firmware_arr[0:CHUNK_SIZE]
            if self.fwVersion is not NO_VERSION:
                fc_corrupt = fc.replace('X', 'A', 2)  # simple corruption; could use a key in the future
            else:
                fc_corrupt = fc  # inital fw released to Bobcat had no version and it didn't support fixing the corrupted header

            self.sock.send(fc_corrupt)
            time.sleep( 0.005 )

            # Get the length of the firmware string in bytes:
            f_size = len( firmware_arr )

            # Send firmware to radar:
            total_chunks = int( math.ceil( f_size / float( CHUNK_SIZE ) ) )
            for i in range(1, total_chunks - 1 ):
                self.overwrite_text_display( "Sending firmware chunk " + str( i + 1 ) + "/" + str( total_chunks ) + " (" +
                                             str( int( 100.0 * float( i + 1 ) / float( total_chunks ) ) ) + "%)" )

                #sys.stdout.write("chunk "+str(i))
                #for b in firmware_arr[i*CHUNK_SIZE:(i+1)*CHUNK_SIZE]:
                #    sys.stdout.write(hex(ord(b))+' ')
                #sys.stdout.write('\n')

                self.sock.send( firmware_arr[i*CHUNK_SIZE:(i+1)*CHUNK_SIZE] )
                self.master.update()
                time.sleep( 0.005 )

            # Send the last chunk which is smaller than the rest:
            #for b in firmware_arr[((total_chunks-1) * CHUNK_SIZE):]:
            #    sys.stdout.write(hex(ord(b))+' ')
            #sys.stdout.write('\n')

            self.sock.send( firmware_arr[((total_chunks-1) * CHUNK_SIZE):] )
            self.master.update()
            time.sleep( 0.005 )

            # Inform the radar that firmware sending is done:
            self.master.update()
            time.sleep( 1.0 )
            self.sock.send( UPDATE_DONE_CMD )

        # Wait to receive end flash message from radar:
        self.update_text_display_newline( "Firmware flash in progress, please wait..." )
        self.sock.settimeout( 120.0 )
        msg = self.sock.recv( 1024 )
        self.sock.settimeout( None )
        self.update_text_display_newline( self.current_radar_ip + " sent: " + msg )

        self.update_text_display_newline( "\n" + "Firmware update complete!" )
        self.update_text_display_newline( "Reboot the radar and restart this GUI, then ensure you can connect with the updated firmware." )

        
    def send_config_to_radar( self ):
        if not self.is_radar_connected:
            self.update_text_display_newline( "ERROR >> Must connect to radar before sending network configuration." )
            return

        # Send start configuration command:
        try:
            self.sock.sendto( CONFIG_CMD, ( self.current_radar_ip, RADAR_PORT ) )
        except socket.error as err:
            self.update_text_display_newline( str(err) )
            return

        time.sleep( 1 )
        msg, addr = self.sock.recvfrom( 1024 )
        self.update_text_display_newline( self.current_radar_ip + " sent: " + msg )

        # Send new network parameters:
        for ind, param in enumerate( self.network_params ):
            self.update_text_display_newline( "Setting " + param[1] + " to: " + self.network_param_entry[param[0]].get() )
            self.sock.sendto( self.network_param_entry[param[0]].get(), ( self.current_radar_ip, RADAR_PORT ) )
            msg, addr = self.sock.recvfrom( 1024 )
            self.update_text_display_newline( addr[0] + " set " + param[1] + " to " + msg )

        self.update_text_display_newline( "Configuration complete!" )
        self.update_text_display_newline( "Reboot the radar and restart this GUI, then ensure you can connect with the updated network configuration." )


    def send_params_to_radar( self ):
        if not self.is_radar_connected:
            self.update_text_display_newline( "ERROR >> Must connect to radar before sending parameters." )
            return
        
        # Send start configuration command:
        try:
            self.sock.sendto( PARAMS_CMD, ( self.current_radar_ip, RADAR_PORT ) )
        except socket.error as err:
            self.update_text_display_newline( str(err) )
            return
        
        time.sleep( 1 )
        msg, addr = self.sock.recvfrom( 1024 )
        self.update_text_display_newline( self.current_radar_ip + " sent: " + msg )

        # Send new filter parameters:
        params_msg = struct.pack( "<BBBBBBBBB", int( 1.0 * float( self.filter_param_entry[self.filter_params[0][0]].get() ) ), \
                                  int( 10.0 * float( self.filter_param_entry[self.filter_params[1][0]].get() ) ), \
                                  int( 10.0 * float( self.filter_param_entry[self.filter_params[2][0]].get() ) ), \
                                  int( float( self.filter_param_entry[self.filter_params[3][0]].get() ) ), \
                                  int( 10.0 * float( self.filter_param_entry[self.filter_params[4][0]].get() ) ), \
                                  int( 10.0 * float( self.filter_param_entry[self.filter_params[5][0]].get() ) ), \
                                  int( 10.0 * float( self.filter_param_entry[self.filter_params[6][0]].get() ) ), \
                                  int( 10.0 * float( self.filter_param_entry[self.filter_params[7][0]].get() ) ), \
                                  int( 10.0 * float( self.filter_param_entry[self.filter_params[8][0]].get() ) ) )
        
        try:
            self.sock.sendto( params_msg, ( self.current_radar_ip, RADAR_PORT ) )
        except socket.error as err:
            self.update_text_display_newline( str(err) )
            return
        
        
    def run_radar( self ):
        self.sock.sendto( RUN_CMD, ( self.current_radar_ip,
                                     RADAR_PORT ) )

        
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
        self.text_display.delete( "end-1l", "end" )
        self.text_display.insert( tk.END, newtext )
        self.text_display.config( state=tk.DISABLED )
        self.text_display.see( "end" )

        
if __name__ == "__main__":
    root = tk.Tk()
    root.geometry( "900x250" )
    root.resizable( 0, 0 )
    app = Window( root )
    root.mainloop()
