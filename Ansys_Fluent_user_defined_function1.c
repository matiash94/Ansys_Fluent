#include "udf.h"
//#include "dynamesh_tools.h"
//#include "unsteady.h"
#include "math.h"
#include <stdio.h>

//real cgx =0;
real cgy =0;
real tiempo = 1;
real kt0=2.29;//valor de rigidez torsional constante
real kt1=3.88;//valor de rigidez torsional para desacoplar.. calculado para que de Vflutter = 17 m/s = 1,2*14 m/s
real kl=143.4;//valor de k long constante
real kt=2.29;//valor de rig tors dependiente del angulo
real tita1=10*3.1415/180;//10 grados
real tita2=17*3.1415/180;//17 grados

DEFINE_SDOF_PROPERTIES(y_unconstrained35,prop,dt,time,dtime)
{
  //cgx = DT_CG(dt)[0];
  //NV_S(vel,=,0.0);
  //tiempo = CURRENT_TIME;
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
  prop[SDOF_LOAD_F_Y]=-kl*(cgy);
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
    fprintf(str,"\n time: %f, DT_THETA_Z: %f, Mz: %f, posic_Y: %f, Fy: %f", tiempo, DT_THETA(dt)[2],prop[SDOF_LOAD_M_Z],DT_CG(dt)[1],prop[SDOF_LOAD_F_Y]);
    fclose(str);
    }
 
 }