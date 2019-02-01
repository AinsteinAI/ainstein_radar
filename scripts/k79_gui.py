#!/usr/bin/env/python

import sys
import errno

if sys.version_info[0] == 3:
    import tkinter as tk # python 3
else:
    import Tkinter as tk # python 2
    
import socket
import time

RADAR_PORT = 7

CONNECT_CMD = "connect"
CONNECT_RES = "Connection established."

CONFIG_CMD = "config"
CONFIG_RES = "Done."
CONFIG_COMPLETE_RES = "All done."

RUN_CMD = "run"

ENTRY_WIDTH = 15
ENTRY_COLSPAN = 2

class Window( tk.Frame ):
    def __init__( self, master=None ):
        tk.Frame.__init__( self, master )
        self.master = master
        
        self.current_radar_ip = ""
        self.is_radar_connected = False

        self.network_params = ( ( "radar_ip", "Radar IP", "" ),
                                ( "radar_netmask", "Radar Netmask", "255.255.255.0" ),
                                ( "radar_gateway", "Radar Gateway", "0.0.0.0" ),
                                ( "host_ip", "Host PC IP", "" ),
                                ( "host_port", "Host PC Port", "1024" ) )
        self.param_labels = {}
        self.param_entry = {}
        
        self.init_window()
        
    def init_window( self ):
        self.master.title( "K79 Network App" )
        self.grid()

        # Set up the Entry widgets for each network parameter:
        for ind, param in enumerate( self.network_params ):
            self.param_labels[param[0]] = tk.Label( self, text=param[1] )
            self.param_labels[param[0]].grid( row=ind, column=0 )
            
            self.param_entry[param[0]] = tk.Entry( self, width=ENTRY_WIDTH, fg="grey" )
            self.param_entry[param[0]].grid( row=ind, column=1, columnspan=ENTRY_COLSPAN )
            self.param_entry[param[0]].delete( 0, tk.END )
            self.param_entry[param[0]].insert( 0, param[2] )
            self.param_entry[param[0]].config( state=tk.DISABLED )

        # Set the Radar IP entry widget to ENABLED and give it focus:
        self.param_entry["radar_ip"].config( state=tk.NORMAL, fg="black" )
        self.param_entry["radar_ip"].focus_set()

        # Set the Host IP entry widget to ENABLED and give it focus:
        self.param_entry["host_ip"].config( state=tk.NORMAL, fg="black" )
            
        # Connect Button:
        self.connect_button = tk.Button( self, text="Connect", command=self.connect_to_radar )
        self.connect_button.grid( row=5, column=0 )
        
        # Send Configuration Button:
        self.send_config_button = tk.Button( self, text="Configure", command=self.send_config_to_radar )
        self.send_config_button.grid( row=5, column=1 )
        self.send_config_button.config( state=tk.DISABLED )
        
        # Run Radar Button:
        self.run_radar_button = tk.Button( self, text="Run", command=self.run_radar )
        self.run_radar_button.grid( row=5, column=2 )
        self.run_radar_button.config( state=tk.DISABLED )
        
        # Display for output:
        self.text_display = tk.Text( self, width=65, height=10 )
        self.text_display.grid( row=0, column=3, rowspan=6 )
        self.update_text_display( "Please enter the IP addresses of your radar and PC, then connect." )

        # Scrollbar for display text:
        self.scrollbar_display = tk.Scrollbar( self, command=self.text_display.yview )
        self.scrollbar_display.grid( row=0, column=4, rowspan=6 )
        self.text_display['yscrollcommand'] = self.scrollbar_display.set
        
    def connect_to_radar( self ):
        if not self.is_radar_connected:
            # Create a new socket instance:
            self.sock = socket.socket( socket.AF_INET, # Internet
                                       socket.SOCK_DGRAM ) # UDP
            self.sock.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )

            # Set the socket timeout:
            self.sock.settimeout( 3 )

            # Bind the socket to the specified host IP address (port doesn't really matter for now):
            try:
                self.sock.bind( ( self.param_entry["host_ip"].get(), int( self.param_entry["host_port"].get() ) ) )
            except socket.error as err:
                if err.errno == errno.EADDRNOTAVAIL:
                    self.update_text_display( "ERROR >> Address not available. Is " + self.param_entry["host_ip"].get() + " the IP of your PC?" )
                else:
                    self.update_text_display( "ERROR >> " + str( err ) )
                return

            # Send the connect command and receive a response:
            self.current_radar_ip = self.param_entry["radar_ip"].get()

            if not self.param_entry["host_ip"].get() or not self.param_entry["radar_ip"].get():
                self.update_text_display( "Please enter the IP addresses of your radar and PC." )
                return
            
            self.sock.sendto( CONNECT_CMD, ( self.current_radar_ip,
                                             RADAR_PORT ) )
            try:
                msg, addr = self.sock.recvfrom( 1024 )
                if msg == CONNECT_RES:
                    self.update_text_display( addr[0] + " sent: " + msg )
                    self.is_radar_connected = True

                    # Enable the entry widgets and buttons:
                    [p.config( state=tk.NORMAL ) for p in self.param_entry]
                    self.send_config_button.config( state=tk.NORMAL )
                    self.run_radar_button.config( state=tk.NORMAL )
                else:
                    self.update_text_display( "ERROR >> Failed to connect to radar." )
            except socket.timeout as err:
                self.update_text_display( "ERROR >> Connect timed out. Is the radar connected and powered?" )
                return

    def send_config_to_radar( self ):
        if not self.is_radar_connected:
            self.update_text_display( "ERROR >> Must connect to radar before sending network configuration." )
            return
        
        # Send start configuration command:
        try:
            self.sock.sendto( CONFIG_CMD, ( self.current_radar_ip, RADAR_PORT ) )
        except socket.error as err:
            self.update_text_display( str(err) )
            return
        
        time.sleep( 1 )
        msg, addr = self.sock.recvfrom( 1024 )
        self.update_text_display( addr[0] + " sent: " + msg )

        # Send new network parameters:
        for ind, param in enumerate( self.network_params ):
            self.update_text_display( "Setting " + param[1] + " to: " + self.param_entry[param[0]].get() )
            self.sock.sendto( self.param_entry[param[0]].get(), ( self.current_radar_ip, RADAR_PORT ) )
            msg, addr = self.sock.recvfrom( 1024 )
            self.update_text_display( addr[0] + " sent: " + msg )
                
        # # Send new radar IP address:
        # self.update_text_display( "Setting radar IP to: " + self.param_entry["radar_ip"].get() )
        # self.sock.sendto( self.param_entry["radar_ip"].get(), ( self.current_radar_ip, RADAR_PORT ) )
        # msg, addr = self.sock.recvfrom( 1024 )
        # if msg == CONFIG_RES:
        #     self.update_text_display( addr[0] + " sent: " + msg )
        # else:
        #     self.update_text_display( "ERROR >> Failed to configure radar IP." )

        # # Send new radar netmask:
        # self.update_text_display( "Setting radar netmask to: " + self.param_entry["radar_netmask"].get() )
        # self.sock.sendto( self.param_entry["radar_netmask"].get(), ( self.current_radar_ip, RADAR_PORT ) )
        # msg, addr = self.sock.recvfrom( 1024 )
        # if msg == CONFIG_RES:
        #     self.update_text_display( addr[0] + " sent: " + msg )
        # else:
        #     self.update_text_display( "ERROR >> Failed to configure radar netmask." )

        # # Send new radar gateway:
        # self.update_text_display( "Setting radar gateway to: " + self.param_entry["radar_gateway"].get() )
        # self.sock.sendto( self.param_entry["radar_gateway"].get(), ( self.current_radar_ip, RADAR_PORT ) )
        # msg, addr = self.sock.recvfrom( 1024 )
        # if msg == CONFIG_RES:
        #     self.update_text_display( addr[0] + " sent: " + msg )
        # else:
        #     self.update_text_display( "ERROR >> Failed to configure radar gateway." )

        # # Send new host IP address:
        # self.update_text_display( "Setting host IP to: " + self.param_entry["host_ip"].get() )
        # self.sock.sendto( self.param_entry["host_ip"].get(), ( self.current_radar_ip, RADAR_PORT ) )
        # msg, addr = self.sock.recvfrom( 1024 )
        # if msg == CONFIG_RES:
        #     self.update_text_display( addr[0] + " sent: " + msg )
        # else:
        #     self.update_text_display( "ERROR >> Failed to configure host IP." )

        # # Send new host IP port:
        # self.update_text_display( "Setting host port to: " + self.param_entry["host_port"].get() )
        # self.sock.sendto( self.param_entry["host_port"].get(), ( self.current_radar_ip, RADAR_PORT ) )
        # msg, addr = self.sock.recvfrom( 1024 )
        # if msg == CONFIG_COMPLETE_RES:
        #     self.update_text_display( addr[0] + " sent: " + msg )
        # else:
        #     self.update_text_display( "ERROR >> Failed to configure host port." )

    def run_radar( self ):
        self.sock.sendto( RUN_CMD, ( self.current_radar_ip,
                                     RADAR_PORT ) )

    def update_text_display( self, newtext ):
        self.text_display.config( state=tk.NORMAL )
        self.text_display.insert( tk.END, newtext+"\n" )
        self.text_display.config( state=tk.DISABLED )
        self.text_display.see( "end" )
        
if __name__ == "__main__":
    root = tk.Tk()
    root.geometry( "715x150" )
    root.resizable( 0, 0 )
    app = Window( root )
    root.mainloop()

    
