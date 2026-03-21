/*******************************************************************************
 * @author  Joseph Kamel, Maxime Georges
 * @email   josephekamel@gmail.com, maxime.georges059@gmail.com
 * @date    10/06/2021
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __VEINS_GeneralLib_H_
#define __VEINS_GeneralLib_H_

#include <random>
#include <omnetpp.h>
#include <string>
// Used in randomization
#include <ctime>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/variate_generator.hpp>
using namespace std;
//using namespace boost;

class GeneralLib {
public:
    GeneralLib();
    GeneralLib(unsigned long seed);

    double GaussianRandomDouble(double mean, double stddev);

    double RandomDouble(double fMin, double fMax);
    int RandomInt(int min, int max);

private:
    boost::random::mt19937 rng;
};

#endif
