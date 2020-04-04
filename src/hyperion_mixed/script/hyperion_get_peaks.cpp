# include "hyperion_get_peaks.h"

void hyperion_operation::networkconfiguration(std::string  ipAddress="10.0.41.1", std::string instrumentName="Hyperion")
{
  try
  {
      hypInst = new Hyperion(ipAddress);
  }

  catch (std::exception &e)
  {
      std::cout << e.what() << std::endl;
      std::cout << "\033[31m [WARN] Hyperion connection failed \033[0m""" << std::endl;
      exit(1);
  }
}

void hyperion_operation::get_peaks(std::vector<double> allPeaks, std::vector<double> wavelength)
{
  // use API to get peaks
  hACQPeaks peaks = hypInst->get_peaks();

  // std::cout << "Total Number of peaks:  " << peaks.get_num_peaks() << std::endl;

  // std::vector<double> allPeaks;
  allPeaks = peaks.get_all();


  double startWavelength, deltaWavelength;
  int numWavelengths;
  hypInst->get_scan_parameters(startWavelength, deltaWavelength, numWavelengths);

  // std::vector<double> wavelength;
  for(int i = 0; i < numWavelengths; i++)
  {
      wavelength.push_back(startWavelength + i*deltaWavelength);
  }
}

void hyperion_operation::test_print()
{
  std::cout << "class test success!" << std::endl;
}
