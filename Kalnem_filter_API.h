//Kalnem_filter_API.c

extern float g_approximation;
extern float g_prevVal      ;
extern float g_coefficient  ;

void kalmenFilterInit(float coeff) ;
float calcOutput (float measuredValue);

