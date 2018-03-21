//Kalnem_filter_API.c

#include "Kalnem_filter_API.h"


float g_approximation=0;
float g_prevVal      =0;
float g_coefficient  =0.15;

void kalmenFilterInit(float coeff) 
{
  g_approximation = 0.0;
  g_prevVal       = 0.0;
  g_coefficient   = coeff;
}



float calcOutput (float measuredValue)
{
  g_approximation = g_coefficient*measuredValue + (1.0-g_coefficient)*g_prevVal;
  g_prevVal       = g_approximation;
  return g_approximation;	
}


