#ifndef CONFIGURATOR_H_
#define CONFIGURATOR_H_

#include <string>

class Configurator {
    private:
        static unsigned short lidarStdev;
        static unsigned short odoStdev;
        static unsigned short compassStdev;

    public:
        Configurator() = delete;
        static void getConfigFromFile(const std::string& filename);
        static unsigned short getLidarStdev();
        static unsigned short getOdoStdev();
        static unsigned short getCompassStdev();

};



#endif