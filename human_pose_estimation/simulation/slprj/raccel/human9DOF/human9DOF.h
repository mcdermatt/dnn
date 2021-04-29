

    /*
  * human9DOF.h
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


  #ifndef RTW_HEADER_human9DOF_h_
  #define RTW_HEADER_human9DOF_h_
  

    

  
#include <stddef.h>

#include <string.h>
  #include "rtw_modelmap.h"
      #ifndef human9DOF_COMMON_INCLUDES_
    # define human9DOF_COMMON_INCLUDES_
    #include <stdlib.h>
    #include "rtwtypes.h"
    #include "simtarget/slSimTgtSigstreamRTW.h"
    #include "simtarget/slSimTgtSlioCoreRTW.h"
    #include "simtarget/slSimTgtSlioClientsRTW.h"
    #include "simtarget/slSimTgtSlioSdiRTW.h"
    #include "sigstream_rtw.h"
    #include "simstruc.h"
    #include "fixedpoint.h"
    #include "raccel.h"
    #include "slsv_diagnostic_codegen_c_api.h"
    #include "rt_logging.h"
    #include "dt_info.h"
    #include "ext_work.h"
    
    #include "nesl_rtw_rtp.h"


    
    #include "human9DOF_836bb176_1_gateway.h"


    #endif /* human9DOF_COMMON_INCLUDES_ */
  

    #include "human9DOF_types.h"    
        
    /* Shared type includes */
          #include "multiword_types.h"

    


  
  
  
  
#include "rt_defines.h"

#include "rtGetInf.h"

#include "rt_nonfinite.h"


  

  

  

        
      #define MODEL_NAME human9DOF
  #define NSAMPLE_TIMES (3) /* Number of sample times */
  #define NINPUTS (0)       /* Number of model inputs */
  #define NOUTPUTS (0)     /* Number of model outputs */
  #define NBLOCKIO (0) /* Number of data output port signals */
  #define NUM_ZC_EVENTS (0) /* Number of zero-crossing events */
  

      #ifndef NCSTATES
  # define NCSTATES (0)   /* Number of continuous states */
  #elif NCSTATES != 0
  # error Invalid specification of NCSTATES defined in compiler command
  #endif
  

    
        #ifndef rtmGetDataMapInfo
    # define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
    #endif

    #ifndef rtmSetDataMapInfo
    # define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
    #endif




  

  

  

          
    /* user code (top of header file) */
    #ifndef IN_RACCEL_MAIN

#endif

      
          /* Block states (default storage) for system '<Root>' */
                
  
     typedef struct   {
  

        
        


        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } egdnkh2kfk; /* '<Root>/To Workspace' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } lwvtrysyed; /* '<Root>/To Workspace1' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } d3kz2ijlqc; /* '<Root>/To Workspace9' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } azyaprarqw; /* '<Root>/To Workspace12' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } jhgo50qk51; /* '<Root>/To Workspace11' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } fppzbt1qdx; /* '<Root>/To Workspace14' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } ka4ky235ay; /* '<Root>/To Workspace13' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } nvsctv0yfd; /* '<Root>/To Workspace15' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } iwsu4x5nwp; /* '<Root>/To Workspace16' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } c3ffwuh404; /* '<Root>/To Workspace17' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } i0gdahuqyw; /* '<Root>/To Workspace18' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } cy5c3cfkqu; /* '<Root>/To Workspace19' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } nnnz24skea; /* '<Root>/To Workspace2' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } bnbmjqfc34; /* '<Root>/To Workspace20' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } lduoghg10r; /* '<Root>/To Workspace21' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } k3edz1q5wy; /* '<Root>/To Workspace22' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } mwdov4uzfg; /* '<Root>/To Workspace23' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } psbc2ehzkf; /* '<Root>/To Workspace24' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } lzbfmnv1s1; /* '<Root>/To Workspace3' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } l4ij2e1qsk; /* '<Root>/To Workspace4' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } nbiaf1cnkk; /* '<Root>/To Workspace5' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } lpqmo0j3x5; /* '<Root>/To Workspace6' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } bqleho13nc; /* '<Root>/To Workspace7' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } cjdp3tu1s3; /* '<Root>/To Workspace8' */
  

      


          
          
          
    



        

          
          

    
            
                  
                
    struct {
        void *LoggedData;
    } g4t2fw4yoc; /* '<Root>/To Workspace10' */
  

      


          
          
          
    



        

          
          

    
            
          
            

              
                          
                   void* fgujja1dx0; /* synthesized block */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   boolean_T eubaypvugh; /* synthesized block */
                          
            
              
             

          
          
          
    

                    
      



  }    DW;
  

        
    
    /* RTWCAPI data structure for RSIM */ 
    typedef struct {
      rtwCAPI_ModelMappingInfo mmi;  
    } DataMapInfo;
      
            /* Parameters (default storage) */
        struct P_ {
          

        
        


        

          
          

    
            
          
            

              
                          
                   real_T j0pi; /* Variable: j0pi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j1pi; /* Variable: j1pi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j1vi; /* Variable: j1vi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j2pi; /* Variable: j2pi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j3ll; /* Variable: j3ll
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j3pi; /* Variable: j3pi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j4pi; /* Variable: j4pi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j5ll; /* Variable: j5ll
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j5pi; /* Variable: j5pi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j5ul; /* Variable: j5ul
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j5vi; /* Variable: j5vi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j6ll; /* Variable: j6ll
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j6pi; /* Variable: j6pi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j6ul; /* Variable: j6ul
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j6vi; /* Variable: j6vi
  * Referenced by:
*   synthesized block
*   synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j7ll; /* Variable: j7ll
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j7pi; /* Variable: j7pi
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    



        

          
          

    
            
          
            

              
                          
                   real_T j7ul; /* Variable: j7ul
  * Referenced by: synthesized block
   */
                          
            
              
             

          
          
          
    

                    
      



        };


  

  

    
    /* External data declarations for dependent source files */
      
      extern const char *RT_MEMORY_ALLOCATION_ERROR; 
        extern DW rtDW; /* states (dwork) */
        
    

              


  extern           P rtP; /* parameters */



      

  

  

      /* Function to get C API Model Mapping Static Info */
    extern const rtwCAPI_ModelMappingStaticInfo*
             human9DOF_GetCAPIStaticMap(void);

      
      /* Simulation Structure */
      extern SimStruct *const rtS;



  

  

  

  

          
    /* user code (bottom of header file) */
    extern const int_T gblNumToFiles;
extern const int_T gblNumFrFiles;
extern const int_T gblNumFrWksBlocks;
extern rtInportTUtable *gblInportTUtables; 
extern const char *gblInportFileName;
extern const int_T gblNumRootInportBlks;
extern const int_T gblNumModelInputs;
extern const int_T gblInportDataTypeIdx[];
extern const int_T gblInportDims[];         
extern const int_T gblInportComplex[];       
extern const int_T gblInportInterpoFlag[];   
extern const int_T gblInportContinuous[];
extern const int_T gblParameterTuningTid;

  extern DataMapInfo* rt_dataMapInfoPtr; 
  extern rtwCAPI_ModelMappingInfo* rt_modelMapInfoPtr;

void MdlOutputs(int_T tid);
void MdlOutputsParameterSampleTime(int_T tid);
void MdlUpdate(int_T tid);
void MdlTerminate(void);
void MdlInitializeSizes(void);
void MdlInitializeSampleTimes(void);
SimStruct * raccel_register_model(void);




    #endif /* RTW_HEADER_human9DOF_h_ */
