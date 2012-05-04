/******************************************************************************
  Target Script for Philips LPC3180 device.

  Copyright (c) 2009 Rowley Associates Limited.

  This file may be distributed under the terms of the License Agreement
  provided with this software.

  THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND, INCLUDING THE
  WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 ******************************************************************************/

var HCLKDIV_CTRL = 0x40004040
var PWR_CTRL = 0x40004044
var HCLKPLL_CTRL = 0x40004058
var SDRAMCLK_CTRL = 0x40004068
var SYSCLK_CTRL = 0x40004050
var DDR_LAP_NOM = 0x4000406C
var DDR_LAP_COUNT = 0x40004070
var EMCControl = 0x31080000
var EMCConfig = 0x31080008
var EMCDynamicControl = 0x31080020
var EMCDynamicRefresh = 0x31080024
var EMCDynamicReadConfig = 0x31080028
var EMCDynamictRP = 0x31080030
var EMCDynamictRAS = 0x31080034
var EMCDynamictSREX = 0x31080038
var EMCDynamictWR = 0x31080044
var EMCDynamictRC = 0x31080048
var EMCDynamictRFC = 0x3108004C
var EMCDynamictXSR = 0x31080050
var EMCDynamictRRD = 0x31080054
var EMCDynamictMRD = 0x31080058
var EMCDynamictCDLR = 0x3108005C
var EMCDynamicRasCas0 = 0x31080104
var EMCDynamicConfig0 = 0x31080100
var EMCStaticConfig0 = 0x31080200
var EMCAHBControl0 = 0x31080400
var EMCCAHBTimeOut0 = 0x31080408
var EMCAHBControl2 = 0x31080440
var EMCCAHBTimeOut2 = 0x31080448
var EMCAHBControl3 = 0x31080460
var EMCCAHBTimeOut3 = 0x31080468
var EMCAHBControl4 = 0x31080480
var EMCCAHBTimeOut4 = 0x31080488

var EMCStaticConfig0 = 0x31080200

function InitDDR()
{
  TargetInterface.pokeWord(PWR_CTRL, 0x00000000);
  TargetInterface.pokeWord(HCLKPLL_CTRL, 0x00000000);
  TargetInterface.pokeWord(SYSCLK_CTRL, 0x00000140);
  TargetInterface.pokeWord(HCLKPLL_CTRL, 0x0001401E);
  TargetInterface.delay(10); // Wait for PLL to Lock
  TargetInterface.pokeWord(HCLKDIV_CTRL, 0x000000BD);
  TargetInterface.pokeWord(PWR_CTRL, 0x00000004);

  TargetInterface.pokeWord(EMCAHBControl0, 0x00000001); // Enable
  TargetInterface.pokeWord(EMCAHBControl3, 0x00000001); // Enable
  TargetInterface.pokeWord(EMCAHBControl4, 0x00000001); // Enable
  TargetInterface.pokeWord(EMCAHBTimeOut0, 0x00000064);
  TargetInterface.pokeWord(EMCAHBTimeOut3, 0x00000190);
  TargetInterface.pokeWord(EMCAHBTimeOut4, 0x00000190);

  TargetInterface.pokeWord(SDRAMCLK_CTRL, 0x00080000);// reset
  TargetInterface.pokeWord(SDRAMCLK_CTRL, 0x0003DE3E);

  TargetInterface.pokeWord(SDRAMCLK_CTRL, 0x0003DF3E);// calibrate
  TargetInterface.pokeWord(SDRAMCLK_CTRL, 0x0003DE3E);
  TargetInterface.delay(1);
  var LAP_COUNT = TargetInterface.peekWord(DDR_LAP_COUNT);
  TargetInterface.pokeWord(DDR_LAP_NOM, LAP_COUNT);

  TargetInterface.pokeWord(EMCControl, 0x00000001); // EMC enabled
  TargetInterface.pokeWord(EMCConfig, 0x00000000); // Little endian mode

  TargetInterface.pokeWord(EMCDynamicConfig0, 0x00001886); // 16-bit low-power DDR SDRAM 512Mb, 32Mx16 | low power DDR SDRAM
  TargetInterface.pokeWord(EMCDynamicRasCas0, 0x00000302);
  TargetInterface.pokeWord(EMCDynamicReadConfig, 0x00000111);

  TargetInterface.pokeWord(EMCDynamictRP, 0x00000003);
  TargetInterface.pokeWord(EMCDynamictRAS, 0x00000005);
  TargetInterface.pokeWord(EMCDynamictSREX, 0x0000000D);
  TargetInterface.pokeWord(EMCDynamictWR, 0x00000002);
  TargetInterface.pokeWord(EMCDynamictRC, 0x00000007);
  TargetInterface.pokeWord(EMCDynamictRFC, 0x00000008);
  TargetInterface.pokeWord(EMCDynamictXSR, 0x0000000D);
  TargetInterface.pokeWord(EMCDynamictRRD, 0x00000002);
  TargetInterface.pokeWord(EMCDynamictMRD, 0x00000001);
  TargetInterface.pokeWord(EMCDynamictCDLR, 0x00000002);

  TargetInterface.pokeWord(EMCDynamicControl, 0x00000183); // SDRAM NOP|CS|CE /*0x00000193); // SDRAM NOP|IMMC|CS|CE*/
  TargetInterface.delay(1);
  TargetInterface.pokeWord(EMCDynamicControl, 0x00000103); // SDRAM PALL|CS|CE /* 0x00000113); // SDRAM PALL|IMMC|CS|CE*/
  TargetInterface.pokeWord(EMCDynamicRefresh, 0x00000002); // CS
  TargetInterface.delay(1);
  TargetInterface.pokeWord(EMCDynamicRefresh, 0x00000030);
  TargetInterface.pokeWord(EMCDynamicControl, 0x00000083); // SDRAM NORMAL|CS|CE /*0x00000093); // SDRAM NORMAL|IMMC|CS|CE*/
  TargetInterface.peekWord(0x80000031);
  TargetInterface.pokeWord(EMCDynamicControl, 0x00000083); // SDRAM NORMAL|CS|CE /*0x00000093); // SDRAM NORMAL|IMMC|CS|CE*/
  TargetInterface.peekWord(0x80004000);
  TargetInterface.pokeWord(EMCDynamicControl, 0x00000000); // SDRAM NORMAL
  
  TargetInterface.pokeWord(SDRAMCLK_CTRL, 0x0003DD3E);// recalibrate
  TargetInterface.pokeWord(SDRAMCLK_CTRL, 0x0003DC3E);
  TargetInterface.delay(1);
}


function ResetOrig()
{
  TargetInterface.resetAndStop(1)
  // turn off the caches
  i = TargetInterface.executeMRC(0xEE010F00);
  TargetInterface.executeMCR(0xEE010F00, i & ~(1<<0|1<<2|1<<12));
  TargetInterface.getICEBreakerRegister(5); /* Clear out Debug Comms Data */
  TargetInterface.pokeWord(EMCStaticConfig0, 0x00000081);
}

function Reset()
{
    TargetInterface.resetAndStop(1)
  // if a normal (not service) boot has occured then the user program may have run so
  // turn off the caches
  i = TargetInterface.executeMRC(0xEE010F00);
  TargetInterface.executeMCR(0xEE010F00, i & ~(1<<0|1<<2|1<<12));
  // clear out debug comms port
  TargetInterface.getICEBreakerRegister(5);
  
  InitDDR();
  TargetInterface.pokeWord(EMCStaticConfig0, 0x00000081);
}