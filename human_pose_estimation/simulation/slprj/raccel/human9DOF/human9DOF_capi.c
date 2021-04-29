    /*
     * human9DOF_capi.c
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

       
  #include "rtw_capi.h"
  
  #ifdef HOST_CAPI_BUILD
  #include "human9DOF_capi_host.h"
  
  #define sizeof(s) ((size_t)(0xFFFF))
  #undef rt_offsetof
  #define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
    
  #define TARGET_CONST
  #define TARGET_STRING(s) (s)    

    
  
  #else /* HOST_CAPI_BUILD */
    
  #include "builtin_typeid_types.h"
  #include "human9DOF.h"       
  #include "human9DOF_capi.h"
    
      #include "human9DOF_private.h"

     

  #ifdef LIGHT_WEIGHT_CAPI
  #define TARGET_CONST                  
  #define TARGET_STRING(s)               (NULL)                    
  #else
  #define TARGET_CONST                   const
  #define TARGET_STRING(s)               (s)
  #endif
    
  #endif /* HOST_CAPI_BUILD */
  

     
  




  
  
  
  
  


		
	/* Block output signal information */
	static const rtwCAPI_Signals rtBlockSignals[] = {

	  /* addrMapIndex, sysNum, blockPath,
	   * signalName, portNumber, dataTypeIndex, dimIndex, fxpIndex, sTimeIndex
	   */

	{
	  0, 0, (NULL), (NULL), 0, 0, 0, 0, 0
	}
      };
	  
	  

	



	static const rtwCAPI_BlockParameters rtBlockParameters[] = {

	  /* addrMapIndex, blockPath,
	   * paramName, dataTypeIndex, dimIndex, fixPtIdx
	   */

	{
	  0, (NULL), (NULL), 0, 0, 0
	}
        };

      
      
       
	   
  

      /* Tunable variable parameters */
      static const rtwCAPI_ModelParameters rtModelParameters[] = {

	/* addrMapIndex, varName, dataTypeIndex, dimIndex, fixPtIndex */

                  
	      {0, TARGET_STRING("j0pi"), 0, 0, 0},
                  
	      {1, TARGET_STRING("j1pi"), 0, 0, 0},
                  
	      {2, TARGET_STRING("j1vi"), 0, 0, 0},
                  
	      {3, TARGET_STRING("j2pi"), 0, 0, 0},
                  
	      {4, TARGET_STRING("j3ll"), 0, 0, 0},
                  
	      {5, TARGET_STRING("j3pi"), 0, 0, 0},
                  
	      {6, TARGET_STRING("j4pi"), 0, 0, 0},
                  
	      {7, TARGET_STRING("j5ll"), 0, 0, 0},
                  
	      {8, TARGET_STRING("j5pi"), 0, 0, 0},
                  
	      {9, TARGET_STRING("j5ul"), 0, 0, 0},
                  
	      {10, TARGET_STRING("j5vi"), 0, 0, 0},
                  
	      {11, TARGET_STRING("j6ll"), 0, 0, 0},
                  
	      {12, TARGET_STRING("j6pi"), 0, 0, 0},
                  
	      {13, TARGET_STRING("j6ul"), 0, 0, 0},
                  
	      {14, TARGET_STRING("j6vi"), 0, 0, 0},
                  
	      {15, TARGET_STRING("j7ll"), 0, 0, 0},
                  
	      {16, TARGET_STRING("j7pi"), 0, 0, 0},
                  
	      {17, TARGET_STRING("j7ul"), 0, 0, 0},
      {0, (NULL), 0, 0, 0 }
      };
	
	

  

  
  
  
  


  
  

        #ifndef HOST_CAPI_BUILD
        /* Declare Data Addresses statically */
	static void*  rtDataAddrMap[] = {
            
            &rtP.j0pi, 	  /* 0: Model Parameter */
            
            
            &rtP.j1pi, 	  /* 1: Model Parameter */
            
            
            &rtP.j1vi, 	  /* 2: Model Parameter */
            
            
            &rtP.j2pi, 	  /* 3: Model Parameter */
            
            
            &rtP.j3ll, 	  /* 4: Model Parameter */
            
            
            &rtP.j3pi, 	  /* 5: Model Parameter */
            
            
            &rtP.j4pi, 	  /* 6: Model Parameter */
            
            
            &rtP.j5ll, 	  /* 7: Model Parameter */
            
            
            &rtP.j5pi, 	  /* 8: Model Parameter */
            
            
            &rtP.j5ul, 	  /* 9: Model Parameter */
            
            
            &rtP.j5vi, 	  /* 10: Model Parameter */
            
            
            &rtP.j6ll, 	  /* 11: Model Parameter */
            
            
            &rtP.j6pi, 	  /* 12: Model Parameter */
            
            
            &rtP.j6ul, 	  /* 13: Model Parameter */
            
            
            &rtP.j6vi, 	  /* 14: Model Parameter */
            
            
            &rtP.j7ll, 	  /* 15: Model Parameter */
            
            
            &rtP.j7pi, 	  /* 16: Model Parameter */
            
            
            &rtP.j7ul, 	  /* 17: Model Parameter */
            
	};
      
        /* Declare Data Run-Time Dimension Buffer Addresses statically */
	static int32_T*  rtVarDimsAddrMap[] = {
	  (NULL)
	};
        #endif
  
 

  /* Data Type Map - use dataTypeMapIndex to access this structure */
  static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap[] = {

    /* cName, mwName, numElements, elemMapIndex, dataSize, slDataId, *
    * isComplex, isPointer, enumStorageType */

      {"double", "real_T", 0, 0, sizeof(real_T), SS_DOUBLE, 0, 0, 0}
  };
  
  #ifdef HOST_CAPI_BUILD
  #undef sizeof
  #endif
  
  /* Structure Element Map - use elemMapIndex to access this structure */
  static TARGET_CONST rtwCAPI_ElementMap rtElementMap[] = {

    /* elementName, elementOffset, dataTypeIndex, dimIndex, fxpIndex */

    {(NULL), 0, 0, 0, 0},
  };

  /* Dimension Map - use dimensionMapIndex to access elements of ths structure*/
  static const rtwCAPI_DimensionMap rtDimensionMap[] = {

    /* dataOrientation, dimArrayIndex, numDims, vardimsIndex */

      {rtwCAPI_SCALAR, 0, 2, 0}
  };

  /* Dimension Array- use dimArrayIndex to access elements of this array */
    static const uint_T rtDimensionArray[] = {
	1,	/* 0 */
      1       /* 1 */
    };

  /* Fixed Point Map */
  static const rtwCAPI_FixPtMap rtFixPtMap[] = {

    /* fracSlopePtr, biasPtr, scaleType, wordLength, exponent, isSigned */

    {(NULL), (NULL), rtwCAPI_FIX_RESERVED, 0, 0, 0 },
  };

  /* Sample Time Map - use sTimeIndex to access elements of ths structure */
  static const rtwCAPI_SampleTimeMap rtSampleTimeMap[] = {

    /* samplePeriodPtr, sampleOffsetPtr, tid, samplingMode */

      {
	(NULL), (NULL), 0, 0
      }
  };

  
  
  
  
  
  

	
	
	

    static rtwCAPI_ModelMappingStaticInfo mmiStatic = {

    /* Signals:{signals, numSignals,
     *           rootInputs, numRootInputs,
     *           rootOutputs, numRootOutputs},
     * Params: {blockParameters, numBlockParameters,
     *          modelParameters, numModelParameters},
     * States: {states, numStates},
     * Maps:   {dataTypeMap, dimensionMap, fixPtMap,
     *          elementMap, sampleTimeMap, dimensionArray},
     * TargetType: targetType
     */

      {rtBlockSignals, 0,
      (NULL), 0,
      (NULL), 0},
      {rtBlockParameters, 0,
      rtModelParameters, 18},
      {(NULL), 0},
    {rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray},
    "float",
    {2526660163U,
    1181451261U,
    771594502U,
    3959330998U},
    (NULL), 0,
	0
    };

    /* Function to get C API Model Mapping Static Info */
    const rtwCAPI_ModelMappingStaticInfo* 
        human9DOF_GetCAPIStaticMap(void) {
          return &mmiStatic;
    }
	
	
  
    
    


  
  
  
    
  #ifndef HOST_CAPI_BUILD
  void human9DOF_InitializeDataMapInfo(void) {
      



  
  /* Set C-API version */
  rtwCAPI_SetVersion((*rt_dataMapInfoPtr).mmi, 1);

  /* Cache static C-API data into the Real-time Model Data structure */
  rtwCAPI_SetStaticMap((*rt_dataMapInfoPtr).mmi, &mmiStatic);

  /* Cache static C-API logging data into the Real-time Model Data structure */
  rtwCAPI_SetLoggingStaticMap((*rt_dataMapInfoPtr).mmi, (NULL));

    /* Cache C-API Data Addresses into the Real-Time Model Data structure */
    rtwCAPI_SetDataAddressMap((*rt_dataMapInfoPtr).mmi, rtDataAddrMap);

    /* Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure */
    rtwCAPI_SetVarDimsAddressMap((*rt_dataMapInfoPtr).mmi, rtVarDimsAddrMap);    
  

  /* Cache the instance C-API logging pointer */
    rtwCAPI_SetInstanceLoggingInfo((*rt_dataMapInfoPtr).mmi, (NULL));

  /* Set reference to submodels */
    rtwCAPI_SetChildMMIArray((*rt_dataMapInfoPtr).mmi, (NULL));
    rtwCAPI_SetChildMMIArrayLen((*rt_dataMapInfoPtr).mmi, 0);


  
  }
  #else /* HOST_CAPI_BUILD */

  #ifdef __cplusplus
  extern "C" {
  #endif

  void human9DOF_host_InitializeDataMapInfo(human9DOF_host_DataMapInfo_T *dataMap, const char *path) {
    
  /* Set C-API version */
  rtwCAPI_SetVersion(dataMap->mmi, 1);

  /* Cache static C-API data into the Real-time Model Data structure */
  rtwCAPI_SetStaticMap(dataMap->mmi, &mmiStatic);

  /* host data address map is NULL */
  rtwCAPI_SetDataAddressMap(dataMap->mmi, NULL);

  /* host vardims address map is NULL */
  rtwCAPI_SetVarDimsAddressMap(dataMap->mmi, NULL);

  /* Set Instance specific path */
  rtwCAPI_SetPath(dataMap->mmi, path);

  rtwCAPI_SetFullPath(dataMap->mmi, NULL);
  
  /* Set reference to submodels */
  
    rtwCAPI_SetChildMMIArray(dataMap->mmi, (NULL));
    rtwCAPI_SetChildMMIArrayLen(dataMap->mmi, 0);
  
  }
  
  #ifdef __cplusplus
  }
  #endif

  #endif /* HOST_CAPI_BUILD */
  
    


     /* EOF: human9DOF_capi.c */
