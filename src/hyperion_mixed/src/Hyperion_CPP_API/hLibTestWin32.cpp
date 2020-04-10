// hLibTestWin32.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "hLibrary.h"
#include "hCommLibrary.h"


int main(int argc, const char * argv[])
{
	//usage hLibTest ipaddress instrumentName

	std::string ipAddress, instrumentName;
	if (argc >= 2)
		ipAddress = argv[1];
	else
		//Hard code a default for development purposes
		ipAddress = "10.0.41.1";

	if (argc == 3)
	{
		instrumentName = argv[2];
	}
	else
	{
		instrumentName = "Hyperion_2";
	}

	std::string newIPAddress = "10.0.41.2";

	std::cout << "Testing creation of Hyperion object" << std::endl;
	Hyperion* hypInst;

	try
	{

		hypInst = new Hyperion(ipAddress);

	}

	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
		exit(1);
	}


	try
	{
		std::string fwVersion, fPGAVersion, hLibraryVersion, serialNumber;
		std::cout << "Getting Version information" << std::endl;

		serialNumber = hypInst->get_serial_number();

		hypInst->get_version(fwVersion, fPGAVersion, hLibraryVersion);

		std::cout << "Serial Number: " << serialNumber << std::endl;
		std::cout << "Firmware Version:  " << fwVersion << std::endl;
		std::cout << "FPGA Version:  " << fPGAVersion << std::endl;
		std::cout << "Library Version:  " << hLibraryVersion << std::endl;

		std::cout << "Setting the instrument name" << std::endl;

		hypInst->set_instrument_name(instrumentName);

		std::string readInstrumentName = hypInst->get_instrument_name();

		std::cout << "Instrument Name Read:  " << readInstrumentName << std::endl;


		std::cout << "Testing peak acquisition" << std::endl;

		hACQPeaks peaks = hypInst->get_peaks();

		std::cout << "Total Number of peaks:  " << peaks.get_num_peaks() << std::endl;

		std::vector<double> channelPeaks, allPeaks;

		channelPeaks = peaks.get_channel(1);

		size_t numChannelPeaks = channelPeaks.size();

		std::cout << "Total Number of peaks on channel 1: " << numChannelPeaks << std::endl;
		if (numChannelPeaks)
		{
			std::cout << "First Peak:  " << channelPeaks[0] << std::endl;
			std::cout << "Last Peak:  " << channelPeaks[numChannelPeaks - 1] << std::endl;
		}

		std::cout << "Running peaks.get_all()" << std::endl;

		allPeaks = peaks.get_all();

		std::cout << "Total number of peaks from get_all():  " << allPeaks.size() << std::endl;
		std::cout << "Timestamp:  " << peaks.timeStamp << " Serial Number: " << peaks.serialNumber << std::endl;

		std::cout << "Testing Spectrum Acquisition" << std::endl;

		int channel = 1;
		double startWavelength, deltaWavelength;
		int numWavelengths;
		hypInst->get_scan_parameters(startWavelength, deltaWavelength, numWavelengths);

		hACQSpectrum spectrum = hypInst->get_spectrum(1, &channel);


		std::cout << "Spectrum start Wavelength: " << startWavelength << std::endl;
		std::cout << "Spectrum Wavelength increment:  " << deltaWavelength << std::endl;
		std::cout << "Number of point in spectrum:  " << numWavelengths << std::endl;
		std::cout << "First value in spectrum:  " << spectrum.calibratedSpectrumData[0] << std::endl;

		std::string spectrumFilename = "hypspectrum.txt";
		std::ofstream spectrumFile(spectrumFilename);
		spectrumFile.precision(6);
		spectrumFile << std::fixed;
		double wavelength;
		for (int i = 0; i < numWavelengths; i++)
		{
			wavelength = startWavelength + i*deltaWavelength;
			spectrumFile << wavelength << ", " << spectrum.calibratedSpectrumData[i] << std::endl;

		}
		spectrumFile.close();


		std::cout << "First value in raw spectrum:  " << spectrum.rawSpectrumData[0] << std::endl;


		std::cout << "Testing streaming peaks acquisition:  " << std::endl;
		int streamingDivider = 1;
		hypInst->enable_peak_streaming(streamingDivider);
		peaks = hypInst->stream_peaks();
		uint64_t serialNum = peaks.serialNumber;
		uint64_t lastSN = serialNum;
		int iterations = 20;
		bool dropped = false;

		for (int i = 0; i < iterations; i++)
		{
			peaks = hypInst->stream_peaks();
			uint64_t serialNum = peaks.serialNumber;
			if ((serialNum - lastSN) != 1)
			{
				std::cout << "Peak Streaming error.  Serial number difference:  " << serialNum - lastSN << std::endl;
				dropped = true;
			}

			lastSN = serialNum;
		}
		if (!dropped)
		{
			std::cout << "No dropped samples in peak acquisition" << std::endl;
		}

		std::cout << "Testing laser scan speed setting" << std::endl;

		int oldSpeed = hypInst->get_laser_scan_speed();

		std::cout << "Current laser scan speed: " << oldSpeed << " Hz." << std::endl;

		std::vector<int> scanSpeeds = hypInst->get_available_laser_scan_speeds();

		std::cout << "Available laser scan speeds: ";
		int newSpeed = 100;
		bool goodSpeed = false;
		for (int speed : scanSpeeds)
		{
			std::cout << speed << " ";

			if (speed == newSpeed)
			{
				goodSpeed = true;
			}
		}

		//10 Hz Instruments cannot run at 100 Hz, so if 100 is not one of the available speeds, then 10 is the proper speed

		if (!goodSpeed)
		{
			newSpeed = 10;
		}
		std::cout << std::endl;

		std::cout << "Setting laser scan speed to " << newSpeed << " Hz." << std::endl;

		hypInst->set_laser_scan_speed(newSpeed);

		newSpeed = hypInst->get_laser_scan_speed();

		std::cout << "Laser scan speed is now: " << newSpeed << " Hz." << std::endl;

		std::cout << "Testing streaming spectrum." << std::endl;


		hypInst->enable_spectrum_streaming(streamingDivider);

		iterations = 20;
		dropped = false;
		//If scanSpeed is 100, then 10 scans are averaged and returned at 10 Hz.
		int expectedSNStep = 1;
		if (newSpeed == 100)
		{
			expectedSNStep = 10;
		}

		spectrum = hypInst->stream_spectrum();
		lastSN = spectrum.spectrumHeader->serialNumber;

		for (int i = 0; i < iterations; i++)
		{
			spectrum = hypInst->get_spectrum();
			uint64_t serialNum = spectrum.spectrumHeader->serialNumber;
			if ((serialNum - lastSN) != expectedSNStep)
			{
				std::cout << "Spectrum Streaming error.  Serial number difference:  " << serialNum - lastSN << std::endl;
				dropped = true;
			}

			lastSN = serialNum;
		}
		if (!dropped)
		{
			std::cout << "No dropped samples in spectrum acquisition" << std::endl;
		}

		std::cout << "Returning laser scan speed to " << oldSpeed << " Hz." << std::endl;

		hypInst->set_laser_scan_speed(oldSpeed);

		newSpeed = hypInst->get_laser_scan_speed();

		std::cout << "Laser scan speed is now: " << newSpeed << " Hz." << std::endl;


		std::cout << "Testing Net API" << std::endl;

		hNetworkSettings netSettings;

		netSettings = hypInst->get_active_network_settings();

		hNetworkSettings initialSettings = netSettings;

		std::cout << "Active Network Settings:  " << std::endl;
		std::cout << "Network Address:  " << netSettings.ipAddress << std::endl;
		std::cout << "Net Mask:  " << netSettings.mask << std::endl;
		std::cout << "Gateway:  " << netSettings.gateway << std::endl;
		std::string ipMode = hypInst->get_network_ip_mode();
		std::cout << "IP Mode:  " << ipMode << std::endl;
		std::cout << "Static Network settings:  " << std::endl;
		netSettings = hypInst->get_static_network_settings();



		hNetworkSettings initialStaticSettings = netSettings;

		std::cout << "Network Address:  " << netSettings.ipAddress << std::endl;
		std::cout << "Net Mask:  " << netSettings.mask << std::endl;
		std::cout << "Gateway:  " << netSettings.gateway << std::endl;

		std::cout << "Changing static IP Address" << std::endl;





		netSettings.ipAddress = newIPAddress;

		hypInst->set_static_network_settings(netSettings.ipAddress, netSettings.mask, netSettings.gateway);

		//This will invalidate the current connection if the IP mode is already static.
		if (ipMode == "STATIC")
		{
			delete hypInst;

			sleep(5);

		}

		else
		{

			netSettings = hypInst->get_static_network_settings();

			std::cout << "Network Address:  " << netSettings.ipAddress << std::endl;
			std::cout << "Net Mask:  " << netSettings.mask << std::endl;
			std::cout << "Gateway:  " << netSettings.gateway << std::endl;

			std::cout << "Switching to static network mode" << std::endl;

			hypInst->set_network_ip_mode(STATIC);

			delete hypInst;

			sleep(2);
		}


		hypInst = new Hyperion(newIPAddress);

		serialNumber = hypInst->get_serial_number();

		std::cout << "Successfully connected to new IP address.  Serial Number: " << serialNumber << std::endl;

		//Return system to DHCP mode if it was there originally
		if (ipMode == "DHCP")
		{
			std::cout << "Returning to dynamic mode" << std::endl;

			hypInst->set_network_ip_mode(DHCP);
		}



	}

	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
		exit(1);
	}


	return 0;
}

