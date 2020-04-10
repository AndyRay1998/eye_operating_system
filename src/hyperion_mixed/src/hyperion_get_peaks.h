#ifndef _HYPERION_GET_PEAKS_H_
#define _HYPERION_GET_PEAKS_H_
#include "Hyperion_CPP_API/hLibrary.h"

class hyperion_operation
{

private:

  Hyperion *hypInst;


public:

  void networkconfiguration(std::string  ipAddress, std::string instrumentName);

  void get_peaks(std::vector<double> allPeaks, std::vector<double> wavelength);

  void test_print();

};


#endif
