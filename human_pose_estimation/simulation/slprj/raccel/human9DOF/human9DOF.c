

    
  
    /*
    * human9DOF.c
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



    

      #include "rt_logging_mmi.h"
  #include "human9DOF_capi.h"
      #include <math.h>

        #include "human9DOF.h"


        #include "human9DOF_private.h"


  
  #include "human9DOF_dt.h"
        
    /* user code (top of parameter file) */
    









extern void* CreateDiagnosticAsVoidPtr_wrapper(const char* id, int nargs, ...);
RTWExtModeInfo* gblRTWExtModeInfo = NULL;
extern boolean_T gblExtModeStartPktReceived;

void raccelForceExtModeShutdown()
{
     if (!gblExtModeStartPktReceived) 
     { 
         boolean_T stopRequested = false;
         rtExtModeWaitForStartPkt(gblRTWExtModeInfo, 2, &stopRequested); 
     } 
    
     rtExtModeShutdown(2); 
}

  #include "slsv_diagnostic_codegen_c_api.h"
  const int_T gblNumToFiles = 0;
  const int_T gblNumFrFiles = 0;
  const int_T gblNumFrWksBlocks = 0;

#ifdef RSIM_WITH_SOLVER_MULTITASKING
  boolean_T gbl_raccel_isMultitasking = 1;
#else
  boolean_T gbl_raccel_isMultitasking = 0;
#endif

  boolean_T gbl_raccel_tid01eq = 1;
  int_T gbl_raccel_NumST = 3;
  const char_T *gbl_raccel_Version = "10.1 (R2020a) 18-Nov-2019";
  
void raccel_setup_MMIStateLog(SimStruct* S){
#ifdef UseMMIDataLogging
  rt_FillStateSigInfoFromMMI(ssGetRTWLogInfo(S),&ssGetErrorStatus(S));
#else
  UNUSED_PARAMETER(S);
#endif
}

    static DataMapInfo  rt_dataMapInfo;
    DataMapInfo* rt_dataMapInfoPtr = &rt_dataMapInfo;
    rtwCAPI_ModelMappingInfo* rt_modelMapInfoPtr = &(rt_dataMapInfo.mmi);
  
  const char *gblSlvrJacPatternFileName = "slprj\\raccel\\human9DOF\\human9DOF_Jpattern.mat";  
  

    /* Root inports information  */
    const int_T gblNumRootInportBlks = 0;
    const int_T gblNumModelInputs    = 0;
    
    extern rtInportTUtable *gblInportTUtables; 
    extern const char *gblInportFileName;
    extern void* gblAperiodicPartitionHitTimes;

    
    
    
    const int_T gblInportDataTypeIdx[]   = {-1};
    const int_T gblInportDims[]          = {-1} ;                   
    const int_T gblInportComplex[]       = {-1};
    const int_T gblInportInterpoFlag[]   = {-1};    
    const int_T gblInportContinuous[]    = {-1};

    int_T enableFcnCallFlag[]   = {1, 1, 1};
    
      const char* raccelLoadInputsAndAperiodicHitTimes(const char* inportFileName,
                                                          int* matFileFormat) {
          return rt_RapidReadInportsMatFile(inportFileName, matFileFormat, 1);
      }

  

      #include "simstruc.h"
      #include "fixedpoint.h"


  

  

  

  

  

  

  

  

  

  

    
    

  

  
      
    
    
    
    
    
    
    
      /* Block states (default storage) */
                


  DW rtDW;

      
  
      /* Parent Simstruct */
      static SimStruct model_S;
      SimStruct *const rtS = &model_S;


  

  

  

  

      

    

  
  
  
                    
      
	            
          

    

      void MdlStart(void)
  {
  
        
  
NeslRtpManager *manager;
NeslRtpManager *manager_p;
boolean_T tmp;

    
  



                      
     
     
     
    {
      void **slioCatalogueAddr = rt_slioCatalogueAddr();
      void * r2 = (NULL);
      void **pOSigstreamManagerAddr = (NULL);
      const int maxErrorBufferSize = 16384;  
      char errMsgCreatingOSigstreamManager[16384];
      bool errorCreatingOSigstreamManager = false;
      const char * errorAddingR2SharedResource = (NULL);
      *slioCatalogueAddr = rtwGetNewSlioCatalogue(rt_GetMatSigLogSelectorFileName());
      errorAddingR2SharedResource = rtwAddR2SharedResource(rtwGetPointerFromUniquePtr(rt_slioCatalogue()), 1 );
      if (errorAddingR2SharedResource != (NULL)) {
        rtwTerminateSlioCatalogue(slioCatalogueAddr);
        *slioCatalogueAddr = (NULL);
        ssSetErrorStatus(rtS, errorAddingR2SharedResource);
        return;
      }
      r2 = rtwGetR2SharedResource(rtwGetPointerFromUniquePtr(rt_slioCatalogue()));
      pOSigstreamManagerAddr = rt_GetOSigstreamManagerAddr();
      errorCreatingOSigstreamManager = rtwOSigstreamManagerCreateInstance(
        rt_GetMatSigLogSelectorFileName(), 
        r2,
        pOSigstreamManagerAddr,
        errMsgCreatingOSigstreamManager,
        maxErrorBufferSize
        );
      if (errorCreatingOSigstreamManager) {
        *pOSigstreamManagerAddr = (NULL);
        ssSetErrorStatus(rtS, errMsgCreatingOSigstreamManager);
        return;
      }
    }

      
  
   
   
   
  {
    bool externalInputIsInDatasetFormat = false;
    void *pISigstreamManager = rt_GetISigstreamManager();
    rtwISigstreamManagerGetInputIsInDatasetFormat( pISigstreamManager, &externalInputIsInDatasetFormat );
    if (externalInputIsInDatasetFormat) {
    }
      
  }  

    
    
  



                    
                                {
        int_T  dimensions[1] = {1};
          
  
  rtDW.egdnkh2kfk.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j6pf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.egdnkh2kfk.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.lwvtrysyed.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j5pf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.lwvtrysyed.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.d3kz2ijlqc.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j3vf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.d3kz2ijlqc.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.azyaprarqw.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j4vf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.azyaprarqw.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.jhgo50qk51.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j4pf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.jhgo50qk51.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.fppzbt1qdx.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j2vf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.fppzbt1qdx.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.ka4ky235ay.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j2pf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.ka4ky235ay.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.nvsctv0yfd.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j0pf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.nvsctv0yfd.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.iwsu4x5nwp.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j0vf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.iwsu4x5nwp.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.c3ffwuh404.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "x",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.c3ffwuh404.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.i0gdahuqyw.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "y",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.i0gdahuqyw.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.cy5c3cfkqu.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "z",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.cy5c3cfkqu.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.nnnz24skea.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j1pf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.nnnz24skea.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.bnbmjqfc34.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "ang",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.bnbmjqfc34.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.lduoghg10r.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j7pf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.lduoghg10r.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.k3edz1q5wy.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j7vf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.k3edz1q5wy.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.mwdov4uzfg.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j8pf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.mwdov4uzfg.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.psbc2ehzkf.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j8vf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.psbc2ehzkf.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.lzbfmnv1s1.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j5vf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.lzbfmnv1s1.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.l4ij2e1qsk.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j1vf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.l4ij2e1qsk.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.nbiaf1cnkk.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j6vf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.nbiaf1cnkk.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.lpqmo0j3x5.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "ax",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.lpqmo0j3x5.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.bqleho13nc.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "ay",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.bqleho13nc.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.cjdp3tu1s3.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "az",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.cjdp3tu1s3.LoggedData == (NULL)) return;
        }
  
                 {
        int_T  dimensions[1] = {1};
          
  
  rtDW.g4t2fw4yoc.LoggedData = rt_CreateLogVar(
             ssGetRTWLogInfo(rtS),
             ssGetTStart(rtS),
             ssGetTFinal(rtS),
             0.0,
             (&ssGetErrorStatus(rtS)),
             "j3pf",
             SS_DOUBLE,
             0,
             0,
             0,
             1,
             1,
             dimensions,
             NO_LOGVALDIMS,             
             (NULL),
             (NULL),
             0,
             1,
             rtInf,
             1);

          if (rtDW.g4t2fw4yoc.LoggedData == (NULL)) return;
        }
  
 manager_p = nesl_lease_rtp_manager("human9DOF/Solver Configuration_1", 0);
manager = manager_p;
tmp = pointer_is_null(manager_p);
if (tmp) {
    human9DOF_836bb176_1_gateway();
    manager = nesl_lease_rtp_manager("human9DOF/Solver Configuration_1", 0);
}
rtDW.fgujja1dx0 = (void *)manager;
rtDW.eubaypvugh = true;





    
  



        
      }
      



       
    
  

          
      
            

    

      void MdlOutputs(int_T tid)
  {
  
                
    
    
  



              
  
    
    
    
  



    
                 
  


                    UNUSED_PARAMETER(tid);



  
  
    
        }
      






    
  

          
      
            

    

      void MdlOutputsTID2(int_T tid)
  {
  
                
    
real_T *parameterBundle_mRealParameters_mX;
real_T tmp[19];
NeuDiagnosticManager *diag;
NeuDiagnosticTree *diagTree;
boolean_T ok;
char *msg;
NeParameterBundle expl_temp;

    
  

          
  
    
    
    
  


                   
                              
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.egdnkh2kfk.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.lwvtrysyed.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.d3kz2ijlqc.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.azyaprarqw.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.jhgo50qk51.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.fppzbt1qdx.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.ka4ky235ay.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.nvsctv0yfd.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.iwsu4x5nwp.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.c3ffwuh404.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.i0gdahuqyw.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.cy5c3cfkqu.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.nnnz24skea.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.bnbmjqfc34.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.lduoghg10r.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.k3edz1q5wy.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.mwdov4uzfg.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.psbc2ehzkf.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.lzbfmnv1s1.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.l4ij2e1qsk.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.nbiaf1cnkk.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.lpqmo0j3x5.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.bqleho13nc.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.cjdp3tu1s3.LoggedData, u, 0);

	  }
    }

   
  
            
    {
      real_T u[1];


        {
          int32_T i;
          for (i = 0; i < 1; i++) {
              u[i] = 0.0;
          }
        }

	  if (ssGetLogOutput(rtS)) {        
      
	  rt_UpdateLogVar((LogVar *)(LogVar*)rtDW.g4t2fw4yoc.LoggedData, u, 0);

	  }
    }

   
  
if (rtDW.eubaypvugh) {
    tmp[0] = rtP.j5ll;
    tmp[1] = rtP.j5ul;
    tmp[2] = rtP.j5vi;
    tmp[3] = rtP.j7pi;
    tmp[4] = rtP.j3pi;
    tmp[5] = rtP.j3ll;
    tmp[6] = rtP.j2pi;
    tmp[7] = rtP.j6pi;
    tmp[8] = rtP.j4pi;
    tmp[9] = rtP.j6ll;
    tmp[10] = rtP.j6ul;
    tmp[11] = rtP.j6vi;
    tmp[12] = rtP.j1pi;
    tmp[13] = rtP.j0pi;
    tmp[14] = rtP.j5pi;
    tmp[15] = rtP.j7ll;
    tmp[16] = rtP.j7ul;
    tmp[17] = rtP.j6vi;
    tmp[18] = rtP.j1vi;
    parameterBundle_mRealParameters_mX = &tmp[0];
    diag = rtw_create_diagnostics();
    diagTree = neu_diagnostic_manager_get_initial_tree(diag);
    expl_temp.mRealParameters.mN = 19;
    expl_temp.mRealParameters.mX = parameterBundle_mRealParameters_mX;
    expl_temp.mLogicalParameters.mN = 0;
    expl_temp.mLogicalParameters.mX = NULL;
    expl_temp.mIntegerParameters.mN = 0;
    expl_temp.mIntegerParameters.mX = NULL;
    expl_temp.mIndexParameters.mN = 0;
    expl_temp.mIndexParameters.mX = NULL;
    ok = nesl_rtp_manager_set_rtps((NeslRtpManager *)rtDW.fgujja1dx0, ssGetT(rtS), expl_temp, diag);
    if (!ok) {
        ok = error_buffer_is_empty(ssGetErrorStatus(rtS));
        if (ok) {
            msg = rtw_diagnostics_msg(diagTree);
            ssSetErrorStatus(rtS, msg);
        }
    }
}
rtDW.eubaypvugh = false;





    
  

                UNUSED_PARAMETER(tid);



  
  
    
        }
      








       
    
  

          
      
            

    

      void MdlUpdate(int_T tid)
  {
  
      
        
      
    
  



                
      
    
  



                    
  



                  UNUSED_PARAMETER(tid);



  

            }
      






    
  

          
      
            

    

      void MdlUpdateTID2(int_T tid)
  {
  
      
        
      
    
  

            
      
    
  

                
  

              UNUSED_PARAMETER(tid);



  

            }
      








            
          

    

      void MdlTerminate(void)
  {
  
        
        
    
    
  



                
  



                  
            
                




          if(rt_slioCatalogue() != (NULL)){
            void **slioCatalogueAddr = rt_slioCatalogueAddr();
            rtwSaveDatasetsToMatFile(rtwGetPointerFromUniquePtr(rt_slioCatalogue()), rt_GetMatSigstreamLoggingFileName());
            rtwTerminateSlioCatalogue(slioCatalogueAddr);
            *slioCatalogueAddr = NULL;
          }
        

    
  



          }
      



      
    


 void MdlInitializeSizes(void)
{
    ssSetNumContStates(rtS, 0); /* Number of continuous states */
  ssSetNumY(rtS, 0);  /* Number of model outputs */
  ssSetNumU(rtS, 0);    /* Number of model inputs */
    ssSetDirectFeedThrough(rtS, 0);  /* The model is not direct feedthrough */
  ssSetNumSampleTimes(rtS, 2);   /* Number of sample times */
  ssSetNumBlocks(rtS, 124); /* Number of blocks */
    ssSetNumBlockIO(rtS, 0); /* Number of block outputs */
    ssSetNumBlockParams(rtS, 18);  /* Sum of parameter "widths" */
}


 void MdlInitializeSampleTimes(void)
{
      /* task periods */
      ssSetSampleTime(rtS, 0, 0.0);
      ssSetSampleTime(rtS, 1, 0.0001);

    /* task offsets */
      ssSetOffsetTime(rtS, 0, 0.0);
      ssSetOffsetTime(rtS, 1, 0.0);
}


  
  
  
 void raccel_set_checksum()
{
    ssSetChecksumVal(rtS, 0, 2526660163U);
  ssSetChecksumVal(rtS, 1, 1181451261U);
  ssSetChecksumVal(rtS, 2, 771594502U);
  ssSetChecksumVal(rtS, 3, 3959330998U);

}


/* Turns off all optimizations on Windows because of issues with VC 2015 compiler.
   This function is not performance-critical, hence this is not a problem.
*/
#if defined(_MSC_VER)
#pragma optimize( "", off )
#endif
 SimStruct * raccel_register_model(void)
{
    static struct _ssMdlInfo mdlInfo;
 
    (void) memset((char *)rtS, 0,
sizeof(SimStruct));
    
      (void) memset((char *)&mdlInfo, 0,
sizeof(struct _ssMdlInfo));
    ssSetMdlInfoPtr(rtS, &mdlInfo);
  
    
    
    /* timing info */
  {
    static time_T mdlPeriod[NSAMPLE_TIMES];
    static time_T mdlOffset[NSAMPLE_TIMES];
    static time_T mdlTaskTimes[NSAMPLE_TIMES];
    static int_T  mdlTsMap[NSAMPLE_TIMES];
    static int_T  mdlSampleHits[NSAMPLE_TIMES];
      static boolean_T  mdlTNextWasAdjustedPtr[NSAMPLE_TIMES];
      static int_T mdlPerTaskSampleHits[NSAMPLE_TIMES * NSAMPLE_TIMES];
      static time_T mdlTimeOfNextSampleHit[NSAMPLE_TIMES];
    
    {
      int_T i;

      for (i = 0; i < NSAMPLE_TIMES; i++) {
	mdlPeriod[i] = 0.0;
	mdlOffset[i] = 0.0;
	mdlTaskTimes[i] = 0.0;
	mdlTsMap[i] = i;
	  mdlSampleHits[i] = 1;
      }
    }

    ssSetSampleTimePtr(rtS, &mdlPeriod[0]);
    ssSetOffsetTimePtr(rtS, &mdlOffset[0]);
    ssSetSampleTimeTaskIDPtr(rtS, &mdlTsMap[0]);
    ssSetTPtr(rtS, &mdlTaskTimes[0]);
    ssSetSampleHitPtr(rtS, &mdlSampleHits[0]);
      ssSetTNextWasAdjustedPtr(rtS, &mdlTNextWasAdjustedPtr[0]);
      ssSetPerTaskSampleHitsPtr(rtS, &mdlPerTaskSampleHits[0]);
      ssSetTimeOfNextSampleHitPtr(rtS, &mdlTimeOfNextSampleHit[0]);
  }
      ssSetSolverMode(rtS, SOLVER_MODE_SINGLETASKING);
    /*
   * initialize model vectors and cache them in SimStruct
   */

    /* states (dwork) */
    {
      void *dwork = (void *) &rtDW;
     
      ssSetRootDWork(rtS, dwork);
        (void) memset(dwork,  0,
 sizeof(DW));
        
    }


    
        /* data type transition information */
      {
          static DataTypeTransInfo dtInfo;

            (void) memset((char_T *) &dtInfo, 0,
sizeof(dtInfo));
          ssSetModelMappingInfo(rtS, &dtInfo);

          dtInfo.numDataTypes  = 14;
            dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
            dtInfo.dataTypeNames = &rtDataTypeNames[0];

            /* Block I/O transition table */
            dtInfo.BTransTable = &rtBTransTable;

            /* Parameters transition table */
            dtInfo.PTransTable = &rtPTransTable;
            dtInfo.dataTypeInfoTable = rtDataTypeInfoTable;
      }
    /* Initialize DataMapInfo substructure containing ModelMap for C API */
      human9DOF_InitializeDataMapInfo();

    ssSetIsRapidAcceleratorActive(rtS, true);
  

  /* Model specific registration */
    ssSetRootSS(rtS, rtS);

    ssSetVersion(rtS, SIMSTRUCT_VERSION_LEVEL2);
  ssSetModelName(rtS, "human9DOF");
  ssSetPath(rtS, "human9DOF");
  
  ssSetTStart(rtS, 0.0);
  
  ssSetTFinal(rtS, 1.0);
  
    ssSetStepSize(rtS, 0.0001);
    ssSetFixedStepSize(rtS, 0.0001);

      
  
  

    
    
  
    

     
     

   
   

  
  

  
      
  /* Setup for data logging */
  {
      static RTWLogInfo rt_DataLoggingInfo;
      rt_DataLoggingInfo.loggingInterval = NULL;

          ssSetRTWLogInfo(rtS, &rt_DataLoggingInfo);

  }


  /* Setup for data logging */
  {

      /*
       * Set pointers to the data and signal info each state
       */
        
  
  


    

        rtliSetLogT(ssGetRTWLogInfo(rtS), "tout");

        rtliSetLogX(ssGetRTWLogInfo(rtS), "");

        rtliSetLogXFinal(ssGetRTWLogInfo(rtS), "");

        rtliSetLogVarNameModifier(ssGetRTWLogInfo(rtS), "none");

        rtliSetLogFormat(ssGetRTWLogInfo(rtS), 4);

        rtliSetLogMaxRows(ssGetRTWLogInfo(rtS), 0);

        rtliSetLogDecimation(ssGetRTWLogInfo(rtS), 1);


    
      

             rtliSetLogY(ssGetRTWLogInfo(rtS), "");


          rtliSetLogYSignalInfo(ssGetRTWLogInfo(rtS), (NULL));

          rtliSetLogYSignalPtrs(ssGetRTWLogInfo(rtS), (NULL));

  }


  
  
      {
    static ssSolverInfo slvrInfo;

    ssSetSolverInfo(rtS, &slvrInfo);
    ssSetSolverName(rtS, "FixedStepDiscrete");
    ssSetVariableStepSolver(rtS, 0);
    ssSetSolverConsistencyChecking(rtS, 0);
    ssSetSolverAdaptiveZcDetection(rtS, 0);
    ssSetSolverRobustResetMethod(rtS, 0);
    
    ssSetSolverStateProjection(rtS, 0);
    ssSetSolverMassMatrixType(rtS, (ssMatrixType)0);
    ssSetSolverMassMatrixNzMax(rtS, 0);       
    ssSetModelOutputs(rtS, MdlOutputs);
    ssSetModelLogData(rtS, rt_UpdateTXYLogVars);
    ssSetModelLogDataIfInInterval(rtS, rt_UpdateTXXFYLogVars);    
    ssSetModelUpdate(rtS, MdlUpdate);
    
    
    ssSetTNextTid(rtS, INT_MIN);
    ssSetTNext(rtS, rtMinusInf);
    ssSetSolverNeedsReset(rtS);
    ssSetNumNonsampledZCs(rtS, 0);
    
    
    
  }




    ssSetChecksumVal(rtS, 0, 2526660163U);
  ssSetChecksumVal(rtS, 1, 1181451261U);
  ssSetChecksumVal(rtS, 2, 771594502U);
  ssSetChecksumVal(rtS, 3, 3959330998U);

      {
          static const sysRanDType  rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
          static RTWExtModeInfo rt_ExtModeInfo;
          static const sysRanDType *systemRan[1];
      
        
          gblRTWExtModeInfo = &rt_ExtModeInfo;
      
        ssSetRTWExtModeInfo(rtS, &rt_ExtModeInfo);
        rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
          
        systemRan[0] = &rtAlwaysEnabled;


      rteiSetModelMappingInfoPtr(ssGetRTWExtModeInfo(rtS), &ssGetModelMappingInfo(rtS));

      rteiSetChecksumsPtr(ssGetRTWExtModeInfo(rtS), ssGetChecksums(rtS));

      rteiSetTPtr(ssGetRTWExtModeInfo(rtS), ssGetTPtr(rtS));
    }

      
            
    
    


  return rtS;
}
/* When you use the on parameter, it resets the optimizations to those that you 
   specified with the /O compiler option. */
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif



  

  

  

      const int_T gblParameterTuningTid = 2;
    void MdlOutputsParameterSampleTime(int_T tid) {
      
      
        MdlOutputsTID2(tid);
    }

