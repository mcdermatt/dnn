

    /*
  * human9DOF_private.h
  *
    * Academic License - for use in teaching, academic research, and meeting
* course requirements at degree granting institutions only.  Not for
* government, commercial, or other organizational use. 
  * 
  * Code generation for model "human9DOF".
  *
  * Model version              : 1.357
  * Simulink Coder version : 10.1 (R2020a) 18-Nov-2019
  * C source code generated on : Wed Apr 28 21:32:55 2021
 * 
 * Target selection: raccel.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
  */


  #ifndef RTW_HEADER_human9DOF_private_h_
  #define RTW_HEADER_human9DOF_private_h_
  

    

      #include "rtwtypes.h"
  #include "builtin_typeid_types.h"
  #include "multiword_types.h"

  
  


  

  

  

    
      #if !defined(rt_VALIDATE_MEMORY)
  #define rt_VALIDATE_MEMORY(S, ptr)   if(!(ptr)) {\
  ssSetErrorStatus(rtS, RT_MEMORY_ALLOCATION_ERROR);\
  }
  #endif
  
  #if !defined(rt_FREE)
  #if !defined(_WIN32)
  #define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((ptr));\
  (ptr) = (NULL);\
  }
  #else
  /* Visual and other windows compilers declare free without const */
  #define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((void *)(ptr));\
  (ptr) = (NULL);\
  }
  #endif
  #endif
  



  

  

  

  

  

  

  

  

  

  

  

  

            
      #if defined(MULTITASKING)
      #  error Model (human9DOF) was built in \
SingleTasking solver mode, however the MULTITASKING define is \
present. If you have multitasking (e.g. -DMT or -DMULTITASKING) \
defined on the Code Generation page of Simulation parameter dialog, please \
remove it and on the Solver page, select solver mode \
MultiTasking. If the Simulation parameter dialog is configured \
correctly, please verify that your template makefile is \
configured correctly.
      #endif


  

  

  

    #endif /* RTW_HEADER_human9DOF_private_h_ */
