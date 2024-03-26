#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <vector>
#include <Widgets.hpp>

class Particle {
    private:
    
    public:
        double weight;
        wxPoint position;
        std::pair<double,double> update;
        std::vector<wxPoint> lidarMeasurements;

        Particle(const wxPoint & position);
        Particle(){};
        void collectMeasurements();

};

#endif