#! /usr/bin/env python3

import numpy as np
from . import hyperion

def getpeaks(current_address='10.0.10.71', ref_wavelen = 1550) -> np.array:

    h = hyperion.Hyperion(current_address)

    ''' We need only 3 channels for eye_op_robot '''
    # spectrum, peaks and wavelengths are array-like data structure
    # We can convert to numpy.array or list
    spectra = h.spectra
    spectrum = spectra[:3]
    peaks = h.peaks[:3] # return wavelengths where peaks are detected
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

if __name__ == '__main__':
    getpeaks()
