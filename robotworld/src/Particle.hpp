#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <vector>
#include <Widgets.hpp>

class Particle {
    private:
    
    public:
        unsigned short weight;
        wxPoint position;
        float compassMeasurement;
        float speedMeasurement;
        std::vector<wxPoint> lidarMeasurements;

        Particle(const wxPoint & position);
        void collectMeasurements();

};

#endif