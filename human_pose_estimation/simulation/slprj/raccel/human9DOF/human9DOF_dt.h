/*
 * human9DOF_dt.h
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

  #include "ext_types.h"

  static DataTypeInfo rtDataTypeInfoTable[] = 
  {
                { "real_T", 0, 8 },
            { "real32_T", 1, 4 },
            { "int8_T", 2, 1 },
            { "uint8_T", 3, 1 },
            { "int16_T", 4, 2 },
            { "uint16_T", 5, 2 },
            { "int32_T", 6, 4 },
            { "uint32_T", 7, 4 },
            { "boolean_T", 8, 1 },
            { "fcn_call_T", 9, 0 },
            { "int_T", 10, 4 },
            { "pointer_T", 11, 8 },
            { "action_T", 12, 8 },
            { "timer_uint32_pair_T", 13, 8 }
  };



  /* data type size table */
  static uint_T rtDataTypeSizes[] = {
              

              sizeof(real_T),
          

              sizeof(real32_T),
          

              sizeof(int8_T),
          

              sizeof(uint8_T),
          

              sizeof(int16_T),
          

              sizeof(uint16_T),
          

              sizeof(int32_T),
          

              sizeof(uint32_T),
          

              sizeof(boolean_T),
          

              sizeof(fcn_call_T),
          

              sizeof(int_T),
          

              sizeof(pointer_T),
          

              sizeof(action_T),
          

              2*sizeof(uint32_T)
  };

  /* data type name table */
  static const char_T * rtDataTypeNames[] = {
    
              "real_T",

              "real32_T",

              "int8_T",

              "uint8_T",

              "int16_T",

              "uint16_T",

              "int32_T",

              "uint32_T",

              "boolean_T",

              "fcn_call_T",

              "int_T",

              "pointer_T",

              "action_T",

              "timer_uint32_pair_T"
  };

  /* data type transitions for block I/O structure */
  static DataTypeTransition rtBTransitions[] = {
    
            



                          {(char_T *)(&rtDW.egdnkh2kfk.LoggedData), 11, 0,              26}              ,
{(char_T *)(&rtDW.eubaypvugh), 8, 0,              1}

    
    
    


  };
  
  /* data type transition table for block I/O structure */
  static DataTypeTransitionTable rtBTransTable = {
                2U,
      rtBTransitions

  };

  /* data type transitions for Parameters structure */
  static DataTypeTransition rtPTransitions[] = {
                              {(char_T *)(&rtP.j0pi), 0, 0,              18}

  };
  
  /* data type transition table for Parameters structure */
  static DataTypeTransitionTable rtPTransTable = {
                1U,
      rtPTransitions

  };

/* [EOF] human9DOF_dt.h */
