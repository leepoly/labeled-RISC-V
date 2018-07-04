// See LICENSE for license details.

package freechips.rocketchip.system

import freechips.rocketchip.config.{Field, Config}
import freechips.rocketchip.subsystem._

case object UseEmu extends Field[Boolean](false)

class WithEmu extends Config ((site, here, up) => {
  case UseEmu => true
})

class LvNAConfigemu extends Config(
  new WithoutFPU
  ++ new WithNonblockingL1(8)
  ++ new WithNBigCores(1)
  ++ new WithEmu
  ++ new WithAsynchronousRocketTiles(8, 3)
  ++ new WithExtMemSize(0x800000L) // 8MB
  ++ new WithNoMMIOPort
  ++ new WithJtagDTM
  ++ new WithDebugSBA
  ++ new BaseConfig)

class LvNAFPGAConfigzedboard extends Config(
  new WithNBigCores(2)
  ++ new WithoutFPU
  ++ new WithAsynchronousRocketTiles(8, 3)
  ++ new WithExtMemSize(0x4000000L) // 64MB
  ++ new WithJtagDTM
  ++ new WithDebugSBA
  ++ new BaseFPGAConfig)

class LvNAFPGAConfigzcu102 extends Config(
  new WithoutFPU
  ++ new WithNonblockingL1(8)
  ++ new WithNBigCores(1)
  ++ new WithAsynchronousRocketTiles(8, 3)
  ++ new WithExtMemSize(0x100000000L)
  ++ new WithJtagDTM
  ++ new WithDebugSBA
  ++ new BaseFPGAConfig)
