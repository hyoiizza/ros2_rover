# Wi-Fi link measured by one robot (rover_multi/wifi_monitor.py), for the link model
# of the coordinator and the link metrics of the experiments.

std_msgs/Header header        # frame_id: world, stamp: measurement time
string robot

# Robot position in world when measured.
geometry_msgs/Point position

# Received signal level of the current access point [dBm]; meaningless when not connected.
float32 rssi_dbm
bool connected
