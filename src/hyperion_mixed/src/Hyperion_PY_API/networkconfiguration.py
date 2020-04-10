#! /usr/bin/env python3
#
#networkconfiguration.py
#
#Copyright (c) 2018 by Micron Optics, Inc.  All Rights Reserved
#
"""This file illustrates how to configure the network settings
"""
from time import sleep
import sys
from . import hyperion
"""This example will change the static IP address, and attempt to connect to
the new address.  If the instrument was originally in DHCP mode, it will
be returned to that mode upon completion.
"""

def hyperion_connect_config(current_address = '10.0.10.71',
                            new_static_address = '10.0.41.3',
                            netmask = '255.255.0.0',
                            gateway = '10.0.0.1'):

    try:
        h1 = hyperion.Hyperion(current_address)

        current_ip_mode = h1.network_ip_mode

        current_static_settings = h1.static_network_settings

        print("Current IP mode: " + current_ip_mode)
        print("Old Static settings: " + repr(current_static_settings)) # repr makes it readable to interpretor

        # Set the desired static settings here
        # Only run the following if you are certain you can connect to the instrument on the new static address

        h1.static_network_settings = hyperion.NetworkSettings(new_static_address, netmask, gateway)

        if current_ip_mode == 'DHCP':
            h1.network_ip_mode = 'STATIC'

        sleep(5)

        h1 = hyperion.Hyperion(new_static_address)

        print(f"\033[1;31;45m Connected with new network settings: {h1.active_network_settings}\033[0m")

    except:
        print("\033[1;32;42m[WARN] Hyperion connection failed!\033[0m")

        sys.exit()


if __name__ == '__main__':
    hyperion_connect_config()
