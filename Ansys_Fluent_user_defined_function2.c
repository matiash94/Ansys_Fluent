#include "udf.h"
//#include "dynamesh_tools.h"
//#include "unsteady.h"
#include "math.h"
#include <stdio.h>

//real cgx =0;
real cgy =0;//posicion vertical del CG.
real tiempo = 1;
real kt0=2.29;//valor de rigidez torsional constante
real kt1=3.88;//valor de rigidez torsional para desacoplar.. calculado para que de Vflutter = 17 m/s = 1,2*14 m/s
real kl=143.4;//valor de k long constante
real kt=2.29;//valor de rig tors dependiente del angulo
real tita1=10*3.1415/180;//10 grados
real tita2=17*3.1415/180;//17 grados

real cgvy=0;//velocidad vertical del CG
real cp=120E-9;//capacitancia
real tita =1.55E-3;//cte de acoplamiento del cosechador
real l=0.5;//longitud l
real R=100E3;//resistencia electrica
real w1=0;//voltage
real f1=0;//usados para integración numérica de ecuac. diferencial con método de Adam-Bashforth
real f2=0;
real f3=0;
real f4=0;
real dt1=2E-4;//paso de tiempo, lo defino porque es cte, y no estoy seguro si es "dt"

DEFINE_SDOF_PROPERTIES(y_unconstrained35,prop,dt,time,dtime)
{
//cgvy=prop[SDOF_VELOCITY_Y];
  cgvy=DT_VEL_CG(dt)[1];
  Message("\n veloc_y: %f, Voltaje: %f, f1: %f, f2: %f, f3: %f, f4: %f\n", cgvy, w1, f1, f2, f3, f4);
  if(CURRENT_TIME<1E-4)
   {
    w1=0;//voltaje inicial
    f1=-tita*0/cp-w1/(cp*R);//es 0
    Message("\n f1 actualizado \n");
    }
    else if((CURRENT_TIME>1E-4)&&(CURRENT_TIME<3E-4))
    {
    f2=-tita*cgvy/cp-w1/(cp*R);
    w1=w1+dt1*f2;
    Message("\n f2 actualizado \n");
    }
    else if((CURRENT_TIME>3E-4)&&(CURRENT_TIME<5E-4))
    {
    f3=-tita*cgvy/cp-w1/(cp*R);
    w1=w1+dt1*f3;
    Message("\n f3 actualizado \n");
    }
    else if((CURRENT_TIME>5E-4)&&(CURRENT_TIME<7E-4))
    {
    f4=-tita*cgvy/cp-w1/(cp*R);
    w1=w1+dt1*f4;
    Message("\n f4 actualizado \n");
    }
    else(CURRENT_TIME>7E-4)
    {
    w1=w1+(dt1/24)*(55.0*f4-59.0*f3+37.0*f2-9.0*f1);
    f1=f2;
    f2=f3;
    f3=f4;
    f4=-tita*cgvy/cp-w1/(cp*R);
    Message("\n Adam-Bashforth\n");
    }
 
  cgy = DT_CG(dt)[1];
  prop[SDOF_MASS]=0.1291;
  prop[SDOF_IZZ]=9.3125E-5;
  prop[SDOF_ZERO_TRANS_X]=TRUE;
  prop[SDOF_ZERO_TRANS_Y]=FALSE;
  prop[SDOF_ZERO_TRANS_Z]=TRUE;
  prop[SDOF_ZERO_ROT_X]=TRUE;
  prop[SDOF_ZERO_ROT_Y]=TRUE;
  prop[SDOF_ZERO_ROT_Z]=FALSE;
  //prop[SDOF_LOAD_F_X]=-1.5*(cgx-0.04);
  prop[SDOF_LOAD_F_Y]=-kl*(cgy)+tita*w1/l;
  if(fabs(DT_THETA(dt)[2])<tita1)
    {
    kt=kt0;
    }
  if(fabs(DT_THETA(dt)[2])>tita1)
    {
    kt=((kt1-kt0)/(tita2-tita1))*(fabs(DT_THETA(dt)[2])-tita1)+kt0;
    }
  prop[SDOF_LOAD_M_Z]=-kt*(DT_THETA(dt)[2])+kl*(cgy)*0.0225;
  //Message("\n time: %f, veloc_y: %f, DT_THETA_Z: %f, Mz: %f, posic_Y: %f\n", tiempo, vel[1], DT_THETA(dt)[2],prop[SDOF_LOAD_M_Z],DT_CG(dt)[1]);
  
  if(tiempo!=CURRENT_TIME)
    {
    tiempo=CURRENT_TIME;
    Message("\n time: %f, DT_THETA_Z: %f, Mz: %f, posic_Y: %f, Fy: %f", tiempo, DT_THETA(dt)[2],prop[SDOF_LOAD_M_Z],DT_CG(dt)[1],prop[SDOF_LOAD_F_Y]);
    FILE *str;
    str=fopen("udf_variables.txt","a");
    //if(str==NULL)
    //{
    //  printf("error\n");
    //}
    //fprintf(str,"\n time: %f, veloc_y: %f, DT_THETA_Z: %f, Mz: %f, posic_Y: %f\n", tiempo, vel[1], DT_THETA(dt)[2],prop[SDOF_LOAD_M_Z],DT_CG(dt)[1]);
    fprintf(str,"\n time: %f, DT_THETA_Z: %f, Mz: %f, posic_Y: %f, Fy: %f, Veloc_y: %f, Voltaje: %f",tiempo, DT_THETA(dt)[2],prop[SDOF_LOAD_M_Z],DT_CG(dt)[1],prop[SDOF_LOAD_F_Y],cgvy,w1);
    fclose(str);
    }
 

 }

