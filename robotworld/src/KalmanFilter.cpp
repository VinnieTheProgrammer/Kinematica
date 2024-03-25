#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(const Matrix<double,4, 1> & initialState, const Matrix<double,4, 1> & initialCovariance): 
currentState(initialState), currentCovariance(initialCovariance), iterated(false) {
}

void KalmanFilter::calcPredictedState() {
    Matrix<double,2,1> cov2(0);
    cov2.at(0,0) = currentCovariance.at(0,0);
    cov2.at(1,0) = currentCovariance.at(1,0);
    predictedState = (calcA() * currentState) + (calcB() * cov2);
}

void KalmanFilter::calcPredictedCovariance() {
    Matrix<double,4,4> covariance(0);
    if(!iterated) {
        covariance.at(0,0) = 1;
        covariance.at(1,1) = 1;
        covariance.at(2,2) = 1;
        covariance.at(3,3) = 1;
    } else {
        covariance.at(0,0) = currentCovariance.at(0,0) + 1;
        covariance.at(1,1) = currentCovariance.at(1,0) + 1;
        covariance.at(2,2) = currentCovariance.at(2,0) + 1;
        covariance.at(3,3) = currentCovariance.at(3,0) + 1;
    }
    Matrix<double,4,4> tempCov = calcA() * covariance * calcA().transpose();
    predictedCovariance.at(0,0) = tempCov.at(0,0);
    predictedCovariance.at(1,1) = tempCov.at(1,1);
    predictedCovariance.at(2,2) = tempCov.at(2,2);
    predictedCovariance.at(3,3) = tempCov.at(3,3);
}

void KalmanFilter::calcKalmanGain() {
    auto thing = predictedCovariance + calcQ();
    kalmanGain.at(0,0) = predictedCovariance.at(0,0) / thing.at(0,0);
    kalmanGain.at(1,1) = predictedCovariance.at(1,1) / thing.at(1,1);
    kalmanGain.at(2,2) = predictedCovariance.at(2,2) / thing.at(2,2);
    kalmanGain.at(3,3) = predictedCovariance.at(3,3) / thing.at(3,3);
}

Matrix<double, 4,1> KalmanFilter::calcAdjustedState(const Matrix<double,4,1> & measurement) {
    Matrix<double, 4,1> adjustedState = predictedState + kalmanGain * (measurement - predictedState);
    return adjustedState;
}

Matrix<double, 4,1> KalmanFilter::calcAdjustedCovariance() {
    Matrix<double,4,4> i(0);
    i.at(0,0) = 1;
    i.at(1,1) = 1;
    i.at(2,2) = 1;
    i.at(3,3) = 1;
    Matrix<double, 4,4> tempCovariance = (i - kalmanGain) * predictedCovariance;
    Matrix<double,4,1> adjustedCovariance(0);
    adjustedCovariance.at(0,0) = tempCovariance.at(0,0);
    adjustedCovariance.at(1,0) = tempCovariance.at(1,1);
    adjustedCovariance.at(2,0) = tempCovariance.at(2,2);
    adjustedCovariance.at(3,0) = tempCovariance.at(3,3);
    return adjustedCovariance;
}

Matrix<double, 4,4> KalmanFilter::calcA() {
    Matrix<double,4,4> A(0);
    A.at(0,0) = 1;
    A.at(1,1) = 1;
    A.at(2,2) = 1;
    A.at(3,3) = 1;
    A.at(2,0) = 1;
    A.at(3,1) = 1;
    return A;
}

Matrix<double, 4,4> KalmanFilter::calcQ() {
    Matrix<double,4,4> Q(0);
    Q.at(0,0) = .1;
    Q.at(1,1) = .1;
    Q.at(2,2) = .1;
    Q.at(3,3) = .1;
    return Q;
}

Matrix<double, 4,2> KalmanFilter::calcB() {
    Matrix<double,4,2> B(0);
    B.at(0,0) = 0.5;
    B.at(1,1) = 0.5;
    B.at(2,0) = 0.5;
    B.at(2,1) = 0.5;
    return B;
}

void KalmanFilter::reset() {
    Matrix<double,4,4> resetGain(0);
    Matrix<double,4,4> resetCov(0);
    Matrix<double,4,1> resetState(0);
    
    predictedState = resetState;
    predictedCovariance = resetCov;
    kalmanGain = resetGain;
}

Matrix<double,2,1> KalmanFilter::executeKalmanFilter(const Matrix<double,4,1> & measurement) {
    calcPredictedState();
    calcPredictedCovariance();

    calcKalmanGain();

    currentState = calcAdjustedState(measurement);
    currentCovariance = calcAdjustedCovariance();

    reset();
    iterated = true;

    Matrix<double,2,1> result(0);
    result.at(0,0) = currentState.at(0,0);
    result.at(1,0) = currentState.at(1,0);
    return result;
}

void KalmanFilter::setCurrentState(const Matrix<double,4, 1> & init) {
    currentState = init;
}

void KalmanFilter::setCurrentCovariance(const Matrix<double,4, 1> & init) {
    currentCovariance = init;
}