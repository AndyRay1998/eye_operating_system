#! /usr/bin/env python
#
#getspectrumandpeaksplot.py
#
#Copyright (c) 2018 by Micron Optics, Inc.  All Rights Reserved
#
"""This example will plot a spectrum and the corresponding peaks.  This
requires numpy, scilab, and matplotlib packages.
"""
# import matplotlib
# matplotlib.use('tkagg') # agg set to enable plotting in tkinter
import numpy as np
# import matplotlib.pyplot as plt
# from scipy.interpolate import interp1d

import sys

from . import hyperion

def getpeaks(current_address='10.0.10.71'):

    h1 = hyperion.Hyperion(current_address)

    ''' We need only 3 channels for eye_op_robot '''
    # peaks = h1.peaks[channel]
    spectra = h1.spectra
    # spectrum = spectra[channel]
    wavelengths = spectra.wavelengths
    result = np.array([h1.peaks[1], h1.peaks[2], h1.peaks[3],
              wavelengths[1], wavelengths[2], wavelengths[3]], dtype=np.float32)

    '''
    #We will interpolate the peak data so that the indicators appear on the
    #plot in line with the spectrum.
    interpSpectrum = interp1d(wavelengths, spectrum)


    plt.plot(wavelengths, spectrum ,peaks,interpSpectrum(peaks),'o') # last two params plot points
    plt.xlabel('Wavelength (nm)')
    plt.ylabel('Amplitude (dBm)')
    plt.show()
    '''
    return result

if __name__ == '__main__':
    getpeaks()
