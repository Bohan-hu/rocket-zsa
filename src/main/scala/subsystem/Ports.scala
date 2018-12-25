// See LICENSE.SiFive for license details.

package freechips.rocketchip.subsystem

import Chisel._
import freechips.rocketchip.amba.ahb._
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.util._

/** Specifies the size and width of external memory ports */
case class MasterPortParams(
  base: BigInt,
  size: BigInt,
  beatBytes: Int,
  idBits: Int,
  maxXferBytes: Int = 256,
  executable: Boolean = true)

/** Specifies the width of external slave ports */
case class SlavePortParams(beatBytes: Int, idBits: Int, sourceBits: Int)

case object ExtMem extends Field[Option[MasterPortParams]](None)
case object ExtBus extends Field[Option[MasterPortParams]](None)
case object ExtIn extends Field[Option[SlavePortParams]](None)

///// The following traits add ports to the sytem, in some cases converting to different interconnect standards

/** Adds a port to the system intended to master an AHB DRAM controller. */
trait CanHaveMasterAHBMemPort { this: BaseSubsystem =>
  val module: CanHaveMasterAHBMemPortModuleImp
  val nMemoryChannels: Int
  private val memPortParamsOpt = p(ExtMem)
  private val portName = "ahb"
  private val device = new MemoryDevice

  require(nMemoryChannels == 0 || memPortParamsOpt.isDefined,
    s"Cannot have $nMemoryChannels with no memory port!")

  val memAHBNode = AHBSlaveNode(Seq.tabulate(nMemoryChannels) { channel =>
    val params = memPortParamsOpt.get
    val base = AddressSet(params.base, params.size-1)
    val filter = AddressSet(channel * cacheBlockBytes, ~((nMemoryChannels-1) * cacheBlockBytes))

    AHBSlavePortParameters(
      slaves = Seq(AHBSlaveParameters(
        address       = base.intersect(filter).toList,
        resources     = device.reg,
        regionType    = RegionType.UNCACHED, // cacheable
        executable    = true,
        supportsWrite = TransferSizes(1, cacheBlockBytes),
        supportsRead  = TransferSizes(1, cacheBlockBytes))),
      beatBytes = params.beatBytes)
  })

  memPortParamsOpt.foreach { params =>
    memBuses.map { m =>
      memAHBNode := m.toDRAMController(Some(portName)) {
        TLToAHB()  // TODO [WHZ] Is this enough?
      }
    }
  }
}

/** Actually generates the corresponding IO in the concrete Module */
trait CanHaveMasterAHBMemPortModuleImp extends LazyModuleImp {
  val outer: CanHaveMasterAHBMemPort

  val mem_ahb = IO(HeterogeneousBag.fromNode(outer.memAHBNode.in))
  (mem_ahb zip outer.memAHBNode.in).foreach { case (io, (bundle, _)) => io <> bundle }
}

/** Adds a AHB port to the system intended to master an MMIO device bus */
trait CanHaveMasterAHBMMIOPort { this: BaseSubsystem =>
  private val mmioPortParamsOpt = p(ExtBus)
  private val portName = "mmio_port_ahb"
  private val device = new SimpleBus(portName.kebab, Nil)

  val mmioAHBNode = AHBSlaveNode(
    mmioPortParamsOpt.map(params =>
      AHBSlavePortParameters(
        slaves = Seq(AHBSlaveParameters(
          address       = AddressSet.misaligned(params.base, params.size),
          resources     = device.ranges,
          executable    = params.executable,
          supportsWrite = TransferSizes(1, params.beatBytes * AHBParameters.maxTransfer),
          supportsRead  = TransferSizes(1, params.beatBytes * AHBParameters.maxTransfer))),
        beatBytes = params.beatBytes)).toSeq)

  mmioPortParamsOpt.map { params =>
    mmioAHBNode := sbus.toFixedWidthPort(Some(portName)) {
      TLToAHB() := TLFragmenter(params.beatBytes, params.beatBytes * AHBParameters.maxTransfer)
    }
  }
}

/** Actually generates the corresponding IO in the concrete Module */
trait CanHaveMasterAHBMMIOPortModuleImp extends LazyModuleImp {
  val outer: CanHaveMasterAHBMMIOPort
  val mmio_ahb = IO(HeterogeneousBag.fromNode(outer.mmioAHBNode.in))

  (mmio_ahb zip outer.mmioAHBNode.in) foreach { case (io, (bundle, _)) => io <> bundle }
}

/** Adds an AHB port to the system intended to be a slave on an MMIO device bus */
trait CanHaveSlaveAHBPort { this: BaseSubsystem =>
  private val slavePortParamsOpt = p(ExtIn)
  private val portName = "slave_port_ahb"
  private val fifoBits = 1

  val l2FrontendAHBNode = AHBMasterNode(
    slavePortParamsOpt.map(params =>
      AHBMasterPortParameters(
        masters = Seq(AHBMasterParameters(
          name = portName.kebab,
          nodePath = Seq()/* TODO [WHZ] Can we ignore this field? */)))).toSeq)

  slavePortParamsOpt.map { params =>
    fbus.fromPort(Some(portName), buffer = BufferParams.default) {
      (TLWidthWidget(params.beatBytes)
        := AHBToTL())
    } := l2FrontendAHBNode
  }
}

/** Actually generates the corresponding IO in the concrete Module */
trait CanHaveSlaveAHBPortModuleImp extends LazyModuleImp {
  val outer: CanHaveSlaveAHBPort
  val l2_frontend_bus_ahb = IO(HeterogeneousBag.fromNode(outer.l2FrontendAHBNode.out).flip)
  (outer.l2FrontendAHBNode.out zip l2_frontend_bus_ahb) foreach { case ((bundle, _), io) => bundle <> io }
}

/** Adds a port to the system intended to master an AXI4 DRAM controller. */
trait CanHaveMasterAXI4MemPort { this: BaseSubsystem =>
  val module: CanHaveMasterAXI4MemPortModuleImp
  val nMemoryChannels: Int
  private val memPortParamsOpt = p(ExtMem)
  private val portName = "axi4"
  private val device = new MemoryDevice

  require(nMemoryChannels == 0 || memPortParamsOpt.isDefined,
    s"Cannot have $nMemoryChannels with no memory port!")

  val memAXI4Node = AXI4SlaveNode(Seq.tabulate(nMemoryChannels) { channel =>
    val params = memPortParamsOpt.get
    val base = AddressSet(params.base, params.size-1)
    val filter = AddressSet(channel * cacheBlockBytes, ~((nMemoryChannels-1) * cacheBlockBytes))

    AXI4SlavePortParameters(
      slaves = Seq(AXI4SlaveParameters(
        address       = base.intersect(filter).toList,
        resources     = device.reg,
        regionType    = RegionType.UNCACHED, // cacheable
        executable    = true,
        supportsWrite = TransferSizes(1, cacheBlockBytes),
        supportsRead  = TransferSizes(1, cacheBlockBytes),
        interleavedId = Some(0))), // slave does not interleave read responses
      beatBytes = params.beatBytes)
  })

  val l2cache: TLSimpleL2Cache = if (p(NL2CacheCapacity) != 0) TLSimpleL2CacheRef() else null
  private val l2node = if (p(NL2CacheCapacity) != 0) l2cache.node else TLSimpleL2Cache()

  memPortParamsOpt.foreach { params =>
    memBuses.map { m =>
       memAXI4Node := m.toDRAMController(Some(portName)) {
        (AXI4Dumper() := AXI4UserYanker() := AXI4IdIndexer(params.idBits) := AXI4AmpDelayer(0.99999, 1000) := TLToAXI4())
      }
    }
  }
}

/** Actually generates the corresponding IO in the concrete Module */
trait CanHaveMasterAXI4MemPortModuleImp extends LazyModuleImp {
  val outer: CanHaveMasterAXI4MemPort

  val mem_axi4 = IO(HeterogeneousBag.fromNode(outer.memAXI4Node.in))
  (mem_axi4 zip outer.memAXI4Node.in).foreach { case (io, (bundle, _)) => io <> bundle }

  def connectSimAXIMem() {
    (mem_axi4 zip outer.memAXI4Node.in).foreach { case (io, (_, edge)) =>
      val mem = LazyModule(new SimAXIMem(edge, size = p(ExtMem).get.size))
      // constrain memory bandwidth 
      val (counterValue, counterWrap) = Counter(true.B, 4)
      val mem_io = Module(mem.module).io.axi4.head
      mem_io <> io
      /*
      mem_io.ar.valid := io.ar.valid && counterWrap
      mem_io.aw.valid := io.aw.valid && counterWrap
      io.ar.ready := mem_io.ar.ready && counterWrap
      io.aw.ready := mem_io.aw.ready && counterWrap
      */
    }
  }
}

/** Adds a AXI4 port to the system intended to master an MMIO device bus */
trait CanHaveMasterAXI4MMIOPort { this: BaseSubsystem =>
  private val mmioPortParamsOpt = p(ExtBus)
  private val portName = "mmio_port_axi4"
  private val device = new SimpleBus(portName.kebab, Nil)

  val mmioAXI4Node = AXI4SlaveNode(
    mmioPortParamsOpt.map(params =>
      AXI4SlavePortParameters(
        slaves = Seq(AXI4SlaveParameters(
          address       = AddressSet.misaligned(params.base, params.size),
          resources     = device.ranges,
          executable    = params.executable,
          supportsWrite = TransferSizes(1, params.maxXferBytes),
          supportsRead  = TransferSizes(1, params.maxXferBytes))),
        beatBytes = params.beatBytes)).toSeq)

  mmioPortParamsOpt.map { params =>
    mmioAXI4Node := sbus.toFixedWidthPort(Some(portName)) {
      (AXI4Buffer()
        := AXI4UserYanker()
        := AXI4Deinterleaver(sbus.blockBytes)
        := AXI4IdIndexer(params.idBits)
        := TLToAXI4())
    }
  }
}

/** Actually generates the corresponding IO in the concrete Module */
trait CanHaveMasterAXI4MMIOPortModuleImp extends LazyModuleImp {
  val outer: CanHaveMasterAXI4MMIOPort
  val mmio_axi4 = IO(HeterogeneousBag.fromNode(outer.mmioAXI4Node.in))

  (mmio_axi4 zip outer.mmioAXI4Node.in) foreach { case (io, (bundle, _)) => io <> bundle }

  def connectSimAXIMMIO() {
    (mmio_axi4 zip outer.mmioAXI4Node.in) foreach { case (io, (_, edge)) =>
      val mmio_mem = LazyModule(new SimAXIMem(edge, size = 4096))
      Module(mmio_mem.module).io.axi4.head <> io
    }
  }
}

/** Adds an AXI4 port to the system intended to be a slave on an MMIO device bus */
trait CanHaveSlaveAXI4Port { this: BaseSubsystem =>
  private val slavePortParamsOpt = p(ExtIn)
  private val portName = "slave_port_axi4"
  private val fifoBits = 1

  val l2FrontendAXI4Node = AXI4MasterNode(
    slavePortParamsOpt.map(params =>
      AXI4MasterPortParameters(
        masters = Seq(AXI4MasterParameters(
          name = portName.kebab,
          id   = IdRange(0, 1 << params.idBits))))).toSeq)

  slavePortParamsOpt.map { params =>
    fbus.fromPort(Some(portName), buffer = BufferParams.default) {
      (TLWidthWidget(params.beatBytes)
        := AXI4ToTL()
        := AXI4UserYanker(Some(1 << (params.sourceBits - fifoBits - 1)))
        := AXI4Fragmenter()
        := AXI4IdIndexer(fifoBits))
    } := l2FrontendAXI4Node
  }
}

/** Actually generates the corresponding IO in the concrete Module */
trait CanHaveSlaveAXI4PortModuleImp extends LazyModuleImp {
  val outer: CanHaveSlaveAXI4Port
  val l2_frontend_bus_axi4 = IO(HeterogeneousBag.fromNode(outer.l2FrontendAXI4Node.out).flip)
  (outer.l2FrontendAXI4Node.out zip l2_frontend_bus_axi4) foreach { case ((bundle, _), io) => bundle <> io }
}

/** Adds a TileLink port to the system intended to master an MMIO device bus */
trait CanHaveMasterTLMMIOPort { this: BaseSubsystem =>
  private val mmioPortParamsOpt = p(ExtBus)
  private val portName = "mmio_port_tl"
  private val device = new SimpleBus(portName.kebab, Nil)

  val mmioTLNode = TLManagerNode(
    mmioPortParamsOpt.map(params =>
      TLManagerPortParameters(
        managers = Seq(TLManagerParameters(
          address            = AddressSet.misaligned(params.base, params.size),
          resources          = device.ranges,
          executable         = params.executable,
          supportsGet        = TransferSizes(1, sbus.blockBytes),
          supportsPutFull    = TransferSizes(1, sbus.blockBytes),
          supportsPutPartial = TransferSizes(1, sbus.blockBytes))),
        beatBytes = params.beatBytes)).toSeq)

  mmioPortParamsOpt.map { params =>
    mmioTLNode := sbus.toFixedWidthPort(Some(portName)) {
      TLBuffer() := TLSourceShrinker(1 << params.idBits)
    }
  }
}


/** Actually generates the corresponding IO in the concrete Module */
trait CanHaveMasterTLMMIOPortModuleImp extends LazyModuleImp {
  val outer: CanHaveMasterTLMMIOPort
  val mmio_tl = IO(HeterogeneousBag.fromNode(outer.mmioTLNode.in))
  (mmio_tl zip outer.mmioTLNode.in) foreach { case (io, (bundle, _)) => io <> bundle }
}

/** Adds an TL port to the system intended to be a slave on an MMIO device bus.
  * NOTE: this port is NOT allowed to issue Acquires.
  */
trait CanHaveSlaveTLPort { this: BaseSubsystem =>
  private val slavePortParamsOpt = p(ExtIn)
  private val portName = "slave_port_tl"

  val l2FrontendTLNode = TLClientNode(
    slavePortParamsOpt.map(params =>
      TLClientPortParameters(
        clients = Seq(TLClientParameters(
          name     = portName.kebab,
          sourceId = IdRange(0, 1 << params.idBits))))).toSeq)

  slavePortParamsOpt.map { params =>
    sbus.fromPort(Some(portName)) {
      TLSourceShrinker(1 << params.sourceBits) := TLWidthWidget(params.beatBytes)
    } := l2FrontendTLNode
  }
}

/** Actually generates the corresponding IO in the concrete Module */
trait CanHaveSlaveTLPortModuleImp extends LazyModuleImp {
  val outer: CanHaveSlaveTLPort
  val l2_frontend_bus_tl = IO(HeterogeneousBag.fromNode(outer.l2FrontendTLNode.out).flip)
  (outer.l2FrontendTLNode.out zip l2_frontend_bus_tl) foreach { case ((bundle, _), io) => bundle <> io }
}

/** Memory with AXI port for use in elaboratable test harnesses. */
class SimAXIMem(edge: AXI4EdgeParameters, size: BigInt)(implicit p: Parameters) extends LazyModule {
  val node = AXI4MasterNode(List(edge.master))

  val sram = LazyModule(new AXI4RAM(AddressSet(0, size-1), beatBytes = edge.bundle.dataBits/8))
  sram.node := AXI4Buffer() := AXI4Fragmenter() := AXI4Delayer(0, 150) := node

  lazy val module = new LazyModuleImp(this) {
    val io = IO(new Bundle { val axi4 = HeterogeneousBag.fromNode(node.out).flip })
    (node.out zip io.axi4) foreach { case ((bundle, _), io) => bundle <> io }
  }
}
