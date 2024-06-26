#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "Particle.hpp"
#include <vector>

class ParticleFilter {
    private:
        static std::vector<Particle> particles;
        Particle previousBestParticle;
        unsigned short particleStartNr; // Number of particles in start pos
        bool initialized;
        unsigned long iterations; 

    public:
        ParticleFilter(const int  & particleStartNr);
        Particle getBestParticle();
        void clearParticles();
        void generateInitialPos(); // initiële random verdeling van particles maken
        void drawParticles(wxDC& dc);

        void updateParticles(); // Update de particles naar de verkregen nieuwe pos van de robot
        void collectMeasurements(); // Krijg alle measurements van de poses
        bool compareMeasurements(); // Vergelijk measurement van particles met robot en geeft gewicht aan overeenkomende filters
        void resample(); // Resample de random verdeling van particles op basis van gewicht
        void executeParticleFilter();
};

#endif