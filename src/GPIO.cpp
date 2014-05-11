#include <fstream>
#include <string>

void initGPIO(int gpioNum, bool direction)
{
	std::ofstream export_ofs("/sys/class/gpio/export");
	export_ofs << std::to_string(gpioNum);
	export_ofs.close();
	
	std::string gpio_direction_file = std::string("/sys/class/gpio") + std::to_string(gpioNum) + std::string("/direction");
	
	std::ofstream direction_ofs(gpio_direction_file);
	direction_ofs << (direction ? "in" : "out") << std::endl;
	direction_ofs.close();
}

char readGPIO(int gpioNum)
{
	char return_val;

	std::string filename = std::string("/sys/class/gpio") + std::to_string(gpioNum) + std::string("/value");

	std::ifstream value_ifs(filename);
	value_ifs >> return_val;
	value_ifs.close();	
	
	return return_val;
}
