#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include "Matrix.hpp"

class KalmanFilter {
    private:
        Matrix<double,4, 1> currentState;
        Matrix<double,4, 1> currentCovariance;

        Matrix<double,4,1> predictedState;
        Matrix<double,4,4> predictedCovariance;

        Matrix<double,4,4> kalmanGain;


        void calcPredictedState();
        void calcPredictedCovariance();
        void calcKalmanGain();
        Matrix<double, 4,1>  calcAdjustedState(const Matrix<double,4,1> & measurement);
        Matrix<double, 4,1> calcAdjustedCovariance();

        Matrix<double, 4,4> calcA();
        Matrix<double, 4,2> calcB();
        Matrix<double, 4,4> calcQ();

        bool iterated;


        void reset(); // Resets variables used in the filter for the next iteration

    public:
        void setCurrentState(const Matrix<double,4, 1> & init);
        void setCurrentCovariance(const Matrix<double,4, 1> & init);

        KalmanFilter(const Matrix<double,4, 1> & initialState, const Matrix<double,4, 1> & initialCovariance);
        Matrix<double,2,1> executeKalmanFilter(const Matrix<double,4,1> & measurement);
};

#endif