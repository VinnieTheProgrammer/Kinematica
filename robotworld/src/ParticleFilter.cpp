#include "ParticleFilter.hpp" 
#include "Particle.hpp" 
#include <random>
#include "RobotWorld.hpp"
#include "Robot.hpp"
#include "Odometer.hpp"
#include "Configurator.hpp"

#include <cmath>

std::vector<Particle> ParticleFilter::particles;
        
ParticleFilter::ParticleFilter(const int  & particleStartNr) : particleStartNr(particleStartNr), initialized(false) {

}

void ParticleFilter::generateInitialPos() {
    if(!initialized) {

        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> noise{0,1024};

        for(int i = 0; i < particleStartNr; ++i) {
            Particle particle(wxPoint(std::abs(noise(gen)), std::abs(noise(gen))));
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

void ParticleFilter::updateParticles() {
    // Move all particles in the direction of the update from the robot. 
    // Take the odometer measurement of the distance/angle and move the particle in that relative direction with a small error (maybe already present)
    for(Particle & particle : particles) {
        wxPoint updatedLocation = calcUpdatedParticleLocations(particle);
        particle.position = updatedLocation;
    }
}

void ParticleFilter::collectMeasurements() {
    for(Particle & particle : particles) {
        particle.collectMeasurements();
    }

}

wxPoint ParticleFilter::calcUpdatedParticleLocations(const Particle & particle) {
    Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot("Robot");
	float speed = robot->getSpeed();
	float compassMeasurement = robot->currectCompassAngle;

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> noise{-1,1};

    wxPoint updatedLocation{static_cast< int >( particle.position.x + std::cos( compassMeasurement)* (speed + noise(gen))),
    static_cast< int >( particle.position.y + std::sin( compassMeasurement)* (speed + noise(gen)))};

    return updatedLocation;
}

double dotProduct(const std::vector<wxPoint>& v1, const std::vector<wxPoint>& v2) {
    double result = 0.0;
    for (size_t i = 0; i < v1.size(); ++i) {
        result += v1[i].x * v2[i].x + v1[i].y * v2[i].y;
    }
    return result;
}

double magnitude(const std::vector<wxPoint>& v) {
    double result = 0.0;
    for (const wxPoint& point : v) {
        result += point.x * point.x + point.y * point.y;
    }
    return std::sqrt(result);
}

// Calculate cosine similarity between two vectors (v1 and v2 are vectors of wxPoint objects)
double cosineSimilarity(const std::vector<wxPoint>& v1, const std::vector<wxPoint>& v2) {
    // Check if the vectors have the same size
    if (v1.size() != v2.size()) {
        std::cerr << "Vectors must have the same size. v1: " << v1.size() << " v2: " << v2.size() << std::endl;
        return 0.0;
    }

    double dot = dotProduct(v1, v2);
    double mag1 = magnitude(v1);
    double mag2 = magnitude(v2);

    // Avoid division by zero
    if (mag1 == 0.0 || mag2 == 0.0) {
        return 0.0;
    }

    return dot / (mag1 * mag2);
}

void ParticleFilter::compareMeasurements() {
    collectMeasurements();
    // Compare the lidarscan of the particles with the robot. 

    Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot("Robot");
    auto robotPercepts = robot->currentRadarPointCloud;
    std::vector<wxPoint> robotCloud;
    // convert radarPointCloud to vector<wxPoint>
    for(Model::DistancePercept percept : robotPercepts) {
        robotCloud.push_back(percept.point);
    }
    for(Particle & particle : particles) {
        auto particleCloud = particle.lidarMeasurements;
        for(int i = 0; i < particleCloud.size(); ++i) {
            double simularity = cosineSimilarity(particleCloud, robotCloud);
            std::cout << "simularity: " << simularity << std::endl;
        }
    }
    // required:
    // Sum of lidar endlpoints squared
    // A: robot lidar vector
    // Cosine simularity (?) -> 

}

void ParticleFilter::resample() {
    // resample proportional to weight
}

void ParticleFilter::executeParticleFilter() {
    generateInitialPos();
    updateParticles();
    compareMeasurements();
    //resample();
}