#! /usr/bin/env python3

import numpy as np
import subprocess
import sys
sys.path.append(subprocess.getoutput("rospack find hyperion_mixed") + '/src/Hyperion_PY_API')
import hyperion

class Hcommand():
    def __init__(self, current_address='10.0.10.71', ref_wavelen = 1550):
        self.current_address = current_address
        self.ref_wavelen = ref_wavelen

        self.h = hyperion.Hyperion(current_address)


    def getpeaks(self):
        '''
        Get peaks in wavelength of 3 channels
        We need only 3 channels for eye_op_robot(default using the first 3 channels)
        '''
        # spectrum, peaks and wavelengths are array-like data structure
        # We can convert to numpy.array or list
        spectra = self.h.spectra
        spectrum = spectra[:3]
        peaks = self.h.peaks[:3] # return wavelengths where peaks are detected
        wavelengths = spectra.wavelengths

        # wavelengths and spectrum are dispersed； use them to interpolate
        interpSpectrum1 = interp1d(wavelengths, spectrum[0])
        # pick the largest peak spectrum in wavelength
        # np.where returns a tuple: (np.array([6])), so the '[0][0]'
        wavelength1 = peaks[0][np.where((interpSpectrum1(peaks[0])))[0][0]]
        # Avoid python for loop, cause it is too slow
        interpSpectrum2 = interp1d(wavelengths, spectrum[1])
        wavelength2 = peaks[1][np.where((interpSpectrum2(peaks[1])))[0][0]]
        interpSpectrum3 = interp1d(wavelengths, spectrum[2])
        wavelength3 = peaks[2][np.where((interpSpectrum3(peaks[2])))[0][0]]

        # calculate difference: use -= to decrease memory usage
        wavelength1 -= ref_wavelen
        wavelength2 -= ref_wavelen
        wavelength3 -= ref_wavelen

        return(np.array([wavelength1, wavelength2, wavelength3]))


    def connect_config(self, new_static_address = '10.0.41.3',
                    netmask = '255.255.0.0', gateway = '10.0.0.1'):
        '''
        network setting config
        '''
        try:
            current_ip_mode = self.h.network_ip_mode

            current_static_settings = self.h.static_network_settings

            print("Current IP mode: " + current_ip_mode)
            print("Old Static settings: " + repr(current_static_settings)) # repr makes it readable to interpretor

            # Set the desired static settings here
            # Only run the following if you are certain you can connect to the instrument on the new static address

            self.h.static_network_settings = hyperion.NetworkSettings(new_static_address, netmask, gateway)

            if current_ip_mode == 'DHCP':
                self.h.network_ip_mode = 'STATIC'

            sleep(5)

            self.h = hyperion.Hyperion(new_static_address)

            print(f"\033[1;31;45m Connected with new network settings: {self.h.active_network_settings}\033[0m")

        except:
            # Error handler
            print("\033[1;32;42m[WARN] Hyperion connection failed!\033[0m")

            sys.exit()
