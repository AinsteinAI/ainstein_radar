# This message describes the Wayv Air radar configuration parameters.
string fw_ver           # Firmware Version
string sbl_ver          # Bootloader Version
uint16 id_485           # 485 ID / Radar Comm ID
uint32 baud_485         # Serial Baud Rate
string server_ip        # Server IP address for Station Mode
uint16 server_port      # Server Port for Station Mode
string wifi_name        # WAYV (AP) broadcast Wifi Name
string wifi_pwd         # WAYV (AP) Broadcast Wifi Password
string con_wifi_name    # Station Mode Wifi Name
string con_wifi_pwd     # Station Mode Wifi Password
string dev_id           # Device Serial ID
uint16 wifi_mode        # AP mode (0) or Station mode(1)
uint16 sys_mode         # WAYV Operating Mode - Normal(0) - TLV(2) - TLV_Wifi(4)
uint16 radio_switch     # Radio Inactive (0) - Radio Inactive (1)
uint16 radio_time       #
string[] radar_conf       # Radar Configuration String
uint16 radar_config_len # Number of Radar Config Entries