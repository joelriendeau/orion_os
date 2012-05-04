var EMCStaticConfig0 = 0x31080200

function Reset()
{
  TargetInterface.resetAndStop(1)
  // if a normal (not service) boot has occured then the user program may have run so
  // turn off the caches
  i = TargetInterface.executeMRC(0xEE010F00);
  TargetInterface.executeMCR(0xEE010F00, i & ~(1<<0|1<<2|1<<12));
  // clear out debug comms port
  TargetInterface.getICEBreakerRegister(5);
  // if a service boot has occured then
  //TargetInterface.pokeWord(EMCStaticConfig0, 0x00000081); //  16-bit NOR flash, respect normal byte-lane select behavior
}