    
  #ifndef RTW_HEADER_human9DOF_cap_host_h_
  #define RTW_HEADER_human9DOF_cap_host_h_
  
  #ifdef HOST_CAPI_BUILD
  
  #include "rtw_capi.h"
  #include "rtw_modelmap.h"
        
  
  typedef struct {
        rtwCAPI_ModelMappingInfo mmi;
    
      } human9DOF_host_DataMapInfo_T;
      
  #ifdef __cplusplus
  extern "C" {
  #endif

  void human9DOF_host_InitializeDataMapInfo(human9DOF_host_DataMapInfo_T *dataMap, const char *path);
  
  #ifdef __cplusplus
  }
  #endif
  
  #endif /* HOST_CAPI_BUILD */
  
  #endif /* RTW_HEADER_human9DOF_cap_host_h_ */

  /* EOF: human9DOF_capi_host.h */
