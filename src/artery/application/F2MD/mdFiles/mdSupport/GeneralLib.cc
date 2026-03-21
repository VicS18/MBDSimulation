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

#include "GeneralLib.h"

GeneralLib::GeneralLib(){
    rng.seed(42);
}

GeneralLib::GeneralLib(unsigned long seed){
    rng.seed(seed);
}

double GeneralLib::RandomDouble(double fMin, double fMax)
{
    std::uniform_real_distribution<> one(fMin,fMax);

    double f = one(rng);
    return f;
}

int GeneralLib::RandomInt(int min, int max)
{
    std::uniform_int_distribution<> one(min,max);

    int guess = one(rng);
    return guess;
}
double GeneralLib::GaussianRandomDouble(double mean, double stddev) {

    std::normal_distribution<> d{mean,stddev};
    return d(rng);
}
