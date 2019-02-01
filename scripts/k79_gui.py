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

class Window( tk.Frame ):
    def __init__( self, master=None ):
        tk.Frame.__init__( self, master )
        self.master = master
        
        self.current_radar_ip = ""
        self.is_radar_connected = False

        self.init_window()
        
    def init_window( self ):
        self.master.title( "K79 Network App" )
        self.grid()

        # Host IP:
        self.host_ip_label = tk.Label( self, text="Host PC IP" )
        self.host_ip_label.grid( row=0, column=0 )
        
        self.host_ip_entry = tk.Entry( self, width=ENTRY_WIDTH )
        self.host_ip_entry.grid( row=0, column=1, columnspan=2 )
        self.host_ip_entry.delete( 0, tk.END )
        self.host_ip_entry.insert( 0, "10.0.0.75" )
        
        # Host Port:
        self.host_port_label = tk.Label( self, text="Host PC Port" )
        self.host_port_label.grid( row=1, column=0 )
        
        self.host_port_entry = tk.Entry( self, width=ENTRY_WIDTH )
        self.host_port_entry.grid( row=1, column=1, columnspan=2 )
        self.host_port_entry.delete( 0, tk.END )
        self.host_port_entry.insert( 0, "1024" )
        
        # Radar IP:
        self.radar_ip_label = tk.Label( self, text="Radar IP" )
        self.radar_ip_label.grid( row=2, column=0 )
        
        self.radar_ip_entry = tk.Entry( self, width=ENTRY_WIDTH )
        self.radar_ip_entry.grid( row=2, column=1, columnspan=2 )
        self.radar_ip_entry.delete( 0, tk.END )
        self.radar_ip_entry.insert( 0, "10.0.0.10" )

        # Radar Netmask:
        self.netmask_label = tk.Label( self, text="Radar Netmask" )
        self.netmask_label.grid( row=3, column=0 )

        self.netmask_entry = tk.Entry( self, width=ENTRY_WIDTH )
        self.netmask_entry.grid( row=3, column=1, columnspan=2 )
        self.netmask_entry.delete( 0, tk.END )
        self.netmask_entry.insert( 0, "255.255.255.0" )
        
        # Radar Gateway:
        self.gateway_label = tk.Label( self, text="Radar Gateway" )
        self.gateway_label.grid( row=4, column=0 )

        self.gateway_entry = tk.Entry( self, width=ENTRY_WIDTH )
        self.gateway_entry.grid( row=4, column=1, columnspan=2 )
        self.gateway_entry.delete( 0, tk.END )
        self.gateway_entry.insert( 0, "0.0.0.0" )
        
        # Connect Button:
        self.connect_button = tk.Button( self, text="Connect", command=self.connect_to_radar )
        self.connect_button.grid( row=5, column=0 )
        
        # Send Configuration Button:
        self.send_config_button = tk.Button( self, text="Send", command=self.send_config_to_radar )
        self.send_config_button.grid( row=5, column=1 )

        # Run Radar Button:
        self.send_config_button = tk.Button( self, text="Run", command=self.run_radar )
        self.send_config_button.grid( row=5, column=2 )

        # Display for output:
        self.text_display = tk.Text( self, width=65, height=10 )
        self.text_display.grid( row=0, column=3, rowspan=6 )
        self.update_text_display( "Please enter your PC's IP address and connect." )
        
    def connect_to_radar( self ):
        if not self.is_radar_connected:
            # Create a new socket instance:
            self.sock = socket.socket( socket.AF_INET, # Internet
                                       socket.SOCK_DGRAM ) # UDP
            self.sock.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )

            # Set the socket timeout:
            self.sock.settimeout( 3 )

            # Bind the socket to the specified host IP address:
            try:
                self.sock.bind( ( self.host_ip_entry.get(), int( self.host_port_entry.get() ) ) )
            except socket.error as err:
                if err.errno == errno.EADDRNOTAVAIL:
                    self.update_text_display( "ERROR >> Address not available. Is " + self.host_ip_entry.get() + " the IP of your PC?" )
                else:
                    self.update_text_display( "ERROR >> " + str( err ) )
                return

            # Send the connect command and receive a response:
            self.current_radar_ip = self.radar_ip_entry.get()
            self.sock.sendto( CONNECT_CMD, ( self.current_radar_ip,
                                             RADAR_PORT ) )
            try:
                msg, addr = self.sock.recvfrom( 1024 )
                if msg == CONNECT_RES:
                    self.update_text_display( addr[0] + " sent: " + msg )
                    self.is_radar_connected = True
                else:
                    self.update_text_display( "ERROR >> Failed to connect to radar." )
            except socket.timeout as err:
                self.update_text_display( "ERROR >> " + str( err ) )
                return

    def send_config_to_radar( self ):
        # Send start configuration command:
        self.sock.sendto( CONFIG_CMD, ( self.current_radar_ip, RADAR_PORT ) )
        time.sleep( 1 )
        msg, addr = self.sock.recvfrom( 1024 )
        self.update_text_display( addr[0] + " sent: " + msg )

        # Send new radar IP address:
        self.update_text_display( "Setting radar IP to: " + self.radar_ip_entry.get() )
        self.sock.sendto( self.radar_ip_entry.get(), ( self.current_radar_ip, RADAR_PORT ) )
        msg, addr = self.sock.recvfrom( 1024 )
        if msg == CONFIG_RES:
            self.update_text_display( addr[0] + " sent: " + msg )
        else:
            self.update_text_display( "ERROR >> Failed to configure radar IP." )

        # Send new radar netmask:
        self.update_text_display( "Setting radar netmask to: " + self.netmask_entry.get() )
        self.sock.sendto( self.netmask_entry.get(), ( self.current_radar_ip, RADAR_PORT ) )
        msg, addr = self.sock.recvfrom( 1024 )
        if msg == CONFIG_RES:
            self.update_text_display( addr[0] + " sent: " + msg )
        else:
            self.update_text_display( "ERROR >> Failed to configure radar netmask." )

        # Send new radar gateway:
        self.update_text_display( "Setting radar gateway to: " + self.gateway_entry.get() )
        self.sock.sendto( self.gateway_entry.get(), ( self.current_radar_ip, RADAR_PORT ) )
        msg, addr = self.sock.recvfrom( 1024 )
        if msg == CONFIG_RES:
            self.update_text_display( addr[0] + " sent: " + msg )
        else:
            self.update_text_display( "ERROR >> Failed to configure radar gateway." )

        # Send new host IP address:
        self.update_text_display( "Setting host IP to: " + self.host_ip_entry.get() )
        self.sock.sendto( self.host_ip_entry.get(), ( self.current_radar_ip, RADAR_PORT ) )
        msg, addr = self.sock.recvfrom( 1024 )
        if msg == CONFIG_RES:
            self.update_text_display( addr[0] + " sent: " + msg )
        else:
            self.update_text_display( "ERROR >> Failed to configure host IP." )

        # Send new host IP port:
        self.update_text_display( "Setting host port to: " + self.host_port_entry.get() )
        self.sock.sendto( self.host_port_entry.get(), ( self.current_radar_ip, RADAR_PORT ) )
        msg, addr = self.sock.recvfrom( 1024 )
        if msg == CONFIG_COMPLETE_RES:
            self.update_text_display( addr[0] + " sent: " + msg )
        else:
            self.update_text_display( "ERROR >> Failed to configure host port." )

    def run_radar( self ):
        self.sock.sendto( RUN_CMD, ( self.current_radar_ip,
                                     RADAR_PORT ) )

    def update_text_display( self, newtext ):
        self.text_display.config( state=tk.NORMAL )
        self.text_display.insert( tk.END, newtext+"\n" )
        self.text_display.config( state=tk.DISABLED )
        
        
if __name__ == "__main__":
    root = tk.Tk()
    root.geometry( "700x150" )
    root.resizable( 0, 0 )
    app = Window( root )
    root.mainloop()

    
