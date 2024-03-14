#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "Particle.hpp"
#include <vector>

class ParticleFilter {
    private:
        static std::vector<Particle> particles;
        int particleStartNr; // Number of particles in start pos
        bool initialized;

    public:
        ParticleFilter(const int  & particleStartNr);

        void generateInitialPos(); // initiële random verdeling van particles maken
        void drawParticles(wxDC& dc);

        void updateParticles(); // Update de particles naar de verkregen nieuwe pos van de robot
        void collectMeasurements(); // Krijg alle measurements van de poses
        void compareMeasurements(); // Vergelijk measurement van particles met robot en geeft gewicht aan overeenkomende filters
        void resample(); // Resample de random verdeling van particles op basis van gewicht
        void executeParticleFilter();
        wxPoint calcUpdatedParticleLocations(const Particle & particle);
};

#endif