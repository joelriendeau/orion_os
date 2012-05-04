/*****************************************************************************
  Abort / undef handlers for the NXP LPC3230
 *****************************************************************************/

  .code 32 // set instruction width in bits, thus this is ARM mode
  .global dabort_handler

dabort_handler:
  // Store the APCS registers
  movs pc, lr