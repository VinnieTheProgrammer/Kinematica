#include "ParticleFilter.hpp" 
#include "Particle.hpp" 
#include <random>
#include "RobotWorld.hpp"
#include "Robot.hpp"
#include "Odometer.hpp"
#include "Configurator.hpp"
#include "MathUtils.hpp"
#include <limits.h>

#include <cmath>

std::vector<Particle> ParticleFilter::particles;
        
ParticleFilter::ParticleFilter(const int  & particleStartNr) : particleStartNr(particleStartNr), initialized(false), iterations(0) {

}

void ParticleFilter::generateInitialPos() {
    if(!initialized) {

        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::uniform_int_distribution<std::mt19937::result_type> dist(0,1024);

        for(int i = 0; i < particleStartNr; ++i) {
            wxPoint location = {dist(gen), dist(gen)};
            Particle particle(location);
            particles.push_back(particle);
        }
        initialized = true;
    }
}

void ParticleFilter::drawParticles(wxDC& dc) {
    if(!initialized) {
        generateInitialPos();
    }
    for(Particle & particle : particles) {
        dc.SetPen( wxPen(  "GREEN", 2, wxPENSTYLE_SOLID));
        dc.DrawCircle(particle.position.x, particle.position.y,2);
    }
}

Particle ParticleFilter::getBestParticle() {
    return previousBestParticle;
}

void ParticleFilter::updateParticles() {
    collectMeasurements();
    if(iterations == 0) {
        return; // To ensure the first iterations isn't updated.
    }

    for(Particle particle : particles) {
        auto angle = particle.update.first;
        auto distance = particle.update.second;
    
        wxPoint endpoint{	static_cast< int >( particle.position.x + std::cos( Utils::MathUtils::toRadians(angle))*distance),
                    static_cast< int >( particle.position.y + std::sin( Utils::MathUtils::toRadians(angle))*distance)};
        particle.position = endpoint;
    }
}

void ParticleFilter::collectMeasurements() {
    for(Particle & particle : particles) {
        particle.collectMeasurements();
    }

}

bool ParticleFilter::compareMeasurements() {
    collectMeasurements();
    // Compare the lidarscan of the particles with the robot. 

    Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot("Robot");
    auto robotPercepts = robot->currentRadarPointCloud;
    if(robotPercepts.empty()) {
        return false;
    }

    std::vector<wxPoint> robotCloud;
    // convert radarPointCloud to vector<wxPoint>
    for(Model::DistancePercept percept : robotPercepts) {
        robotCloud.push_back(percept.point);
    }
    double maxDeviation = 0; // Keep a record of the bigest to get a smaller range later on
    for(int i = 0; i < particles.size(); ++i) {
        double deviation = 0;
        auto particleCloud = particles[i].lidarMeasurements;
        for(int j = 0; j < particleCloud.size(); ++j) {
            double xDev = std::abs(robotCloud[j].x - particleCloud[j].x);
            double yDev = std::abs(robotCloud[j].y - particleCloud[j].y);
            deviation += (xDev + yDev);
        }
        particles[i].weight = deviation;

        if(deviation > maxDeviation) {
            maxDeviation = deviation;
        }
    }

    // This ensures we will have a range of 0-1 instead of a huge range of combined deviations
    for(Particle particle: particles) {
        particle.weight /= maxDeviation; 
    }

    return true;
}

bool sortByWeight(const Particle& a, const Particle& b) {
    // Sort based on the 'value' variable
    return a.weight < b.weight;
}

void ParticleFilter::resample() {
    std::sort(particles.begin(), particles.end(), sortByWeight);

    // Take the best particle and use it to resample
    Particle bestParticle = particles[0];
    previousBestParticle = bestParticle; // previous best particle for the update
    particles.clear();

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> newXPosition{bestParticle.position.x,15};
    std::normal_distribution<> newYPosition{bestParticle.position.y,15};
    short nrOfResampledParticles = particleStartNr - (iterations * 25);

    if(nrOfResampledParticles < 100) {
        nrOfResampledParticles = 100;
    }

    for(int i = 0; i < nrOfResampledParticles; ++i) {
        wxPoint location = {newXPosition(gen), newYPosition(gen)};
        Particle particle(location);
        particles.push_back(particle);
    }

}

void ParticleFilter::executeParticleFilter() {
    generateInitialPos();
    updateParticles();
    if(compareMeasurements()) {
        resample();
        iterations++;
    }
}

void ParticleFilter::clearParticles() {
    particles.clear();
}