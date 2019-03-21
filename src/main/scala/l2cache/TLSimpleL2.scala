//**************************************************************************
// L2 Cache
//--------------------------------------------------------------------------
//
// Zhigang Liu
// 2018 Aug 15

package freechips.rocketchip.subsystem

import Chisel._
import chisel3.util.IrrevocableIO
import chisel3.core.{Input, Output}
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.tilelink._
import lvna.{HasControlPlaneParameters, CPToL2CacheIO}

case class TLL2CacheParams(
  debug: Boolean = true //false
)

trait L2CacheParams1 {
  val blockSize: Int = 64 * 8
  val blockBytes: Int = blockSize / 8

}

class MetadataEntry(tagBits: Int, dsidWidth: Int) extends Bundle {
  val valid = Bool()
  val dirty = Bool()
  val tag = UInt(width = tagBits.W)
  val rr_state = Bool()
  val dsid = UInt(width = dsidWidth.W)
  override def cloneType = new MetadataEntry(tagBits, dsidWidth).asInstanceOf[this.type]
}

class L2CacheReq(params: TLBundleParameters, dsidWidth: Int) extends Bundle with L2CacheParams1{
  val opcode = UInt(3.W)
  val dsid = UInt(width = dsidWidth.W)
  val param = UInt(1.W)
  val size = UInt(width = log2Up(log2Ceil(params.dataBits / 8) + log2Ceil(64 * 8 / params.dataBits)).W)
  val source = UInt(width = params.sourceBits.W)
  val address = UInt(width = params.addressBits.W)
  val mask = Vec(8, UInt(8.W))
  val data = Vec(blockSize / params.dataBits, UInt(params.dataBits.W))
  override def cloneType = new L2CacheReq(params, dsidWidth).asInstanceOf[this.type]
}

class L2CacheStageInfo(params: TLBundleParameters, dsidWidth: Int) extends Bundle with L2CacheParams1{
  val req = Output(new L2CacheReq(params, dsidWidth))
  val valid = Output(Bool())
  val ready = Input(Bool())
  override def cloneType = new L2CacheStageInfo(params, dsidWidth).asInstanceOf[this.type]
}

final class BitsSnoop[+T <: Data](gen: T) extends Bundle
{
  val bits = gen.asOutput
  override def cloneType: this.type = new BitsSnoop(gen).asInstanceOf[this.type]
}

class TLCacheConvertorIn(params: TLBundleParameters, dsidWidth: Int) extends Module with L2CacheParams1{
  val io = new Bundle {
    // in.a: L1 ----> Convertor ----> L2
    // in.d: L1 <---- Convertor <---- L2
    val tl_in_a = new BitsSnoop(new TLBundleA(params)).flip
    val tl_in_d = new BitsSnoop(new TLBundleD(params))
    val tl_in_a_valid = Input(Bool())
    val tl_in_a_ready = Output(Bool())
    val tl_in_d_valid = Output(Bool())
    val tl_in_d_ready = Input(Bool())
    val cache_s0 = new L2CacheStageInfo(params, dsidWidth)
    val cache_rst = Bool(INPUT)
    val cache_s3 = new L2CacheStageInfo(params, dsidWidth).flip
  }
  val s0_idle :: s0_gather_write_data :: s0_send_bresp :: s0_wait_cache_ready :: Nil = Enum(UInt(), 4)
  val s0_state = Reg(init = s0_idle)
  val convertor_debug = Bool(true)

  //cache only sees 64B per cycle
  val innerBeatSize = io.tl_in_d.bits.params.dataBits
  val innerBeatBytes = innerBeatSize / 8
  val innerDataBeats = blockSize / innerBeatSize
  val innerBeatBits = log2Ceil(innerBeatBytes)
  val innerBeatIndexBits = log2Ceil(innerDataBeats)
  val innerBeatLSB = innerBeatBits
  val innerBeatMSB = innerBeatLSB + innerBeatIndexBits - 1

  val outerBeatSize = 64 * 8//io.out.d.bits.params.dataBits
  val outerBeatBytes = outerBeatSize / 8
  val outerDataBeats = blockSize / outerBeatSize
  val addrWidth = io.tl_in_a.bits.params.addressBits
  val innerIdWidth = io.tl_in_a.bits.params.sourceBits
  //val split = innerBeatSize / outerBeatSize
  //val splitBits = log2Ceil(split)
  //require(isPow2(split))

  val in_opcode = io.tl_in_a.bits.opcode
  val in_dsid = io.tl_in_a.bits.dsid
  val s0_in_addr = io.tl_in_a.bits.address
  val in_id   = io.tl_in_a.bits.source

  val in_len_shift = io.tl_in_a.bits.size >= innerBeatBits.U
  val in_len  = Mux(in_len_shift, ((1.U << io.tl_in_a.bits.size) >> innerBeatBits) - 1.U, 0.U)  // #word, i.e., arlen in AXI
  val in_data = io.tl_in_a.bits.data
  val in_data_mask = io.tl_in_a.bits.mask

  val in_recv_fire = io.tl_in_a_valid && io.tl_in_a_ready
  val in_read_req = in_recv_fire && (in_opcode === TLMessages.Get)
  val in_write_req = in_recv_fire && (in_opcode === TLMessages.PutFullData || in_opcode === TLMessages.PutPartialData)
  //val in_recv_handshake = io.tl_in_a_ready

  val s0_start_beat = s0_in_addr(innerBeatMSB, innerBeatLSB)
  val s0_inner_end_beat_reg = Reg(UInt(4.W))
  val s0_inner_end_beat = Mux(s0_state === s0_idle, s0_start_beat + in_len, s0_inner_end_beat_reg)
  val gather_curr_beat_reg = RegInit(0.asUInt(log2Ceil(innerDataBeats).W))
  val gather_curr_beat = Mux(s0_state === s0_idle, s0_start_beat, gather_curr_beat_reg)
  val gather_last_beat = gather_curr_beat === s0_inner_end_beat

  val s0_info = Reg(new L2CacheReq(params, dsidWidth))
  val s0_valid = Wire(init = Bool(false))

  val s3_idle :: s3_data_resp :: Nil = Enum(UInt(), 2)
  val s3_state = Reg(init = s3_idle)

  io.cache_s0.req := s0_info
  io.cache_s0.valid := s0_valid

  when (s0_state === s0_idle && s3_state === s3_idle) {
    when (in_read_req) {
      s0_info.address := s0_in_addr
      s0_info.source := in_id
      s0_info.opcode := in_opcode
      s0_info.dsid := in_dsid
      s0_info.size := io.tl_in_a.bits.size
      s0_inner_end_beat_reg := s0_start_beat + in_len

      s0_state := s0_wait_cache_ready
    } .elsewhen (in_write_req) {
      s0_info.address := s0_in_addr
      s0_info.source := in_id
      s0_info.opcode := in_opcode
      s0_info.dsid := in_dsid
      s0_info.size := io.tl_in_a.bits.size
      s0_inner_end_beat_reg := s0_start_beat + in_len

      s0_state := s0_gather_write_data
    // } .elsewhen (io.tl.b.fire() || io.tl.c.fire() || io.tl.e.fire()) {
    //   assert(Bool(false), "Inner tilelink Unexpected handshake")
    }
  }

  when (convertor_debug) {
    when (GTimer() % 10000.U === 0.U) {
      printf("[in.a] cycle: %d a_addr %x a_opcode%x s0_state%x cache_s0.valid%x cache_s0.ready%x a_ready%x a_valid%x a_data %x\n", 
      GTimer(), io.tl_in_a.bits.address,
      io.tl_in_a.bits.opcode, s0_state, io.cache_s0.valid, io.cache_s0.ready,
      io.tl_in_a_ready, io.tl_in_a_valid, 
      io.tl_in_a.bits.data
      )
    }
  }
  
  

  // *** gather_wdata ***
  // s_gather_write_data:
  // gather write data
  when ((in_write_req || (s0_state === s0_gather_write_data && in_recv_fire)) && (s3_state === s3_idle)) {
    when (s0_state === s0_idle) {
      gather_curr_beat_reg := s0_start_beat + 1.U
    } .elsewhen (s0_state === s0_gather_write_data) {
      gather_curr_beat_reg := gather_curr_beat_reg + 1.U
    } .otherwise {
      assert(Bool(false), "state error")
    }

    s0_info.data(gather_curr_beat) := in_data(64 - 1, 0)
    s0_info.mask(gather_curr_beat) := in_data_mask(8 - 1, 0)
    when (gather_last_beat) {
      s0_state := s0_send_bresp
    }
  }

  // s_send_bresp:
  // send bresp, end write transaction
  val in_write_ok = s0_state === s0_send_bresp && s3_state === s3_idle
  val in_send_ok = io.tl_in_d_valid && io.tl_in_d_ready

  when (s0_state === s0_send_bresp && in_send_ok && s3_state === s3_idle) {
    s0_state := s0_wait_cache_ready
  }
  //TODO: input queue could be added here, for non-blocking

  when (s0_state === s0_wait_cache_ready && s3_state === s3_idle) { //won't give req to cache until resp finished
    s0_valid := Bool(true)
    when (io.cache_s0.ready) {
      s0_state := s0_idle
    }
  }

  // *** data resp ***

  val s3_in_addr = io.cache_s3.req.address
  val s3_start_beat = s3_in_addr(innerBeatMSB, innerBeatLSB)
  val s3_inner_end_beat_reg = Reg(UInt(4.W))
  val s3_inner_end_beat = Mux(s3_state === s3_idle, s3_start_beat + 7.U, s3_inner_end_beat_reg)
  io.cache_s3.ready := Mux(s3_state === s3_idle, Bool(true), Bool(false))
  val data_resp_buf = Reg(Vec(8, UInt(64.W)))
  val resp_curr_beat = RegInit(0.asUInt(log2Ceil(innerDataBeats).W))
  val resp_last_beat = resp_curr_beat === s3_inner_end_beat
  val data_resp = Wire(UInt(64.W))
  data_resp := data_resp_buf(resp_curr_beat)

  val in_read_ok = s3_state === s3_data_resp //data_resp is prior than accepting a req

  when (io.cache_s3.valid && s3_state === s3_idle) {
    data_resp_buf := io.cache_s3.req.data
    s3_state := s3_data_resp
    resp_curr_beat := s3_start_beat
    s3_inner_end_beat_reg := s3_start_beat + 7.U
  }
  when (s3_state === s3_data_resp && in_send_ok) {
    resp_curr_beat := resp_curr_beat + 1.U
    when (resp_last_beat) {
      s3_state := s3_idle
    }
  }
  io.tl_in_d_valid := in_write_ok || in_read_ok
  io.tl_in_d.bits.opcode  := Mux(in_read_ok, TLMessages.AccessAckData, TLMessages.AccessAck)
  io.tl_in_d.bits.param   := UInt(0)
  io.tl_in_d.bits.size    := Mux(in_read_ok, io.cache_s3.req.size, s0_info.size)
  io.tl_in_d.bits.source  := Mux(in_read_ok, io.cache_s3.req.source, s0_info.source)
  io.tl_in_d.bits.sink    := UInt(0)
  io.tl_in_d.bits.denied  := Bool(false)
  io.tl_in_d.bits.data    := data_resp
  io.tl_in_d.bits.corrupt := Bool(false)
  
  when (convertor_debug) {
    when ((s3_state =/= s3_idle)) {
      printf("[in.d] cycle: %d d_opcode%x s3_state%x, cache_s3.valid%x d_ready%x d_valid%x d_source%x resp_curr_beat%x d_data %x\n", 
        GTimer(),
        io.tl_in_d.bits.opcode, s3_state, io.cache_s3.valid, 
        io.tl_in_d_ready, io.tl_in_d_valid, 
        io.tl_in_d.bits.source,
        resp_last_beat, data_resp
        )
    }
  }

  val raddr_recv_ready = s0_state === s0_idle && !io.cache_rst && s3_state === s3_idle
  val waddr_recv_ready = s0_state === s0_idle && !io.cache_rst && s3_state === s3_idle
  val wdata_recv_ready = s0_state === s0_gather_write_data && s3_state === s3_idle
  io.tl_in_a_ready := raddr_recv_ready || waddr_recv_ready || wdata_recv_ready
  val raddr_fire = raddr_recv_ready && in_recv_fire
  val waddr_fire = waddr_recv_ready && in_recv_fire
  val wdata_fire = wdata_recv_ready && in_recv_fire
}


class TLCacheConvertorOut(params: TLBundleParameters, dsidWidth: Int) extends Module with L2CacheParams1{
  val io = new Bundle {
    // out.a: L2 ----> ConvertorOut ----> Mem
    // out.d: L2 <---- ConvertorOut <---- Mem
    val tl_out_a = new BitsSnoop(new TLBundleA(params))
    val tl_out_d = new BitsSnoop(new TLBundleD(params)).flip
    val tl_out_a_valid = Output(Bool())
    val tl_out_a_ready = Input(Bool())
    val tl_out_d_valid = Input(Bool())
    val tl_out_d_ready = Output(Bool())
    //writeback
    val wb_addr = Input(UInt(width = params.addressBits.W))
    val wb_data = Input(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
    val wb_valid = Input(Bool())
    val wb_ready = Output(Bool())
    //refill
    val rf_addr = Input(UInt(width = params.addressBits.W))
    val rf_data = Output(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
    val rf_addr_valid = Input(Bool())
    val rf_data_valid = Output(Bool())
    val rf_ready = Input(Bool())
    val dsid = Input(UInt(dsidWidth.W))
  }
  val outerBeatSize = io.tl_out_d.bits.params.dataBits
  val outerDataBeats = blockSize / outerBeatSize
  val outerBeatBytes = outerBeatSize / 8
  val outerBeatLen = log2Ceil(outerBeatBytes)
  val outerBurstLen = outerBeatLen + log2Ceil(outerDataBeats)

  val out_raddr_fire = io.tl_out_a_valid && io.tl_out_a_ready
  val out_waddr_fire = io.tl_out_a_valid && io.tl_out_a_ready
  val out_wdata_fire = io.tl_out_a_valid && io.tl_out_a_ready
  val out_wreq_fire = io.tl_out_d_valid && io.tl_out_d_ready
  val out_rdata_fire = io.tl_out_d_valid && io.tl_out_d_ready
  val out_rdata = io.tl_out_d.bits.data

  val wb_idle :: wb_wait_ram_awready :: wb_do_ram_write :: wb_wait_ram_bresp :: wb_wait_refill :: Nil = Enum(UInt(), 5)
  //without MSHR, we now support simple serial wb+rf
  val rf_idle :: rf_wait_ram_ar :: rf_do_ram_read :: rf_wait_cache_ready :: Nil = Enum(UInt(), 4)
  val wb_state = Reg(init = wb_idle)
  val rf_state = Reg(init = rf_idle)
  val convertor_out_debug = Bool(true)

  //writeback
  val wb_addr_buf = Reg(UInt(width = params.addressBits.W))
  val wb_data_buf = Reg(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
  when (wb_state === wb_idle && io.wb_valid) {
    wb_state := wb_wait_ram_awready
    wb_addr_buf := io.wb_addr
    wb_data_buf := io.wb_data
    when (convertor_out_debug) {
      printf("[out.a] wb cycle: %d, wb_state %x, wb_addr %x, wb_data %x \n",
        GTimer(),
        wb_state,
        io.wb_addr,
        io.wb_data.asUInt
        )
    }
  }
  when (wb_state === wb_wait_ram_awready) {
    wb_state := wb_do_ram_write //there is one useless extra cycle...
  }
  val (wb_cnt, wb_done) = Counter(out_wdata_fire && wb_state === wb_do_ram_write, outerDataBeats)
  when (wb_state === wb_do_ram_write && wb_done) {
    wb_state := wb_wait_ram_bresp
  }
  when (wb_state === wb_wait_ram_bresp && out_wreq_fire) {
    wb_state := wb_wait_refill
  }
  val out_write_valid = wb_state === wb_do_ram_write

  //refill
  val no_wb_req = wb_state === wb_idle || wb_state === wb_wait_refill
  when (rf_state === rf_idle && io.rf_addr_valid && no_wb_req) {
    rf_state := rf_wait_ram_ar
  }
  io.rf_data_valid := rf_state === rf_wait_cache_ready && no_wb_req
  io.wb_ready := wb_state === wb_idle
  val rf_mem_data = Reg(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
  when (rf_state === rf_wait_ram_ar && no_wb_req && out_raddr_fire) {
    rf_state := rf_do_ram_read
  }
  val (rf_mem_cnt, rf_mem_done) = Counter(rf_state === rf_do_ram_read && no_wb_req && out_rdata_fire, outerDataBeats)
  when (rf_state === rf_do_ram_read && no_wb_req && out_rdata_fire) {
    rf_mem_data(rf_mem_cnt) := out_rdata
    when (rf_mem_done) {
      rf_state := rf_wait_cache_ready
    }
  }
  when (rf_state === rf_wait_cache_ready && no_wb_req && io.rf_ready) {
    rf_state := rf_idle
    io.rf_data := rf_mem_data
    when (wb_state === wb_wait_refill) {
      wb_state := wb_idle
    }
  }

  when (convertor_out_debug) {
    when (rf_state =/= rf_idle && rf_state =/= rf_do_ram_read) {
      printf("cycle%d rf_state%x, rf_addr_valid%x, rf_data_valid%x, a_valid%x, a_ready%x, d_valid%x, d_ready%x rf_mem_cnt%x, data %x \n",
        GTimer(),
        rf_state,
        io.rf_addr_valid, io.rf_data_valid,
        io.tl_out_a_valid, io.tl_out_a_ready, io.tl_out_d_valid, io.tl_out_d_ready,
        rf_mem_cnt, 
        io.tl_out_d.bits.data
      )
    }
    when (wb_state =/= wb_idle) {
      printf("cycle%d wb_state%x no_wb_req%x\n",
        GTimer(), wb_state, no_wb_req
      )
    }
  }

  //outer wire
  val out_read_valid = (rf_state === rf_wait_ram_ar) && no_wb_req
  val out_data = wb_data_buf(wb_cnt)
  val out_addr = Mux(out_read_valid, io.rf_addr, wb_addr_buf)
  val out_opcode = Mux(out_read_valid, TLMessages.Get, TLMessages.PutFullData)
  io.tl_out_a.bits.opcode  := out_opcode
  io.tl_out_a.bits.dsid    := io.dsid
  io.tl_out_a.bits.param   := UInt(0)
  io.tl_out_a.bits.size    := outerBurstLen.U
  io.tl_out_a.bits.source  := 0.asUInt(params.sourceBits.W)
  io.tl_out_a.bits.address := out_addr
  io.tl_out_a.bits.mask    := 0xff.U //edgeOut.mask(out_addr, outerBurstLen.U)
  io.tl_out_a.bits.data    := out_data
  io.tl_out_a.bits.corrupt := Bool(false)

  io.tl_out_a_valid := out_write_valid || out_read_valid
  // read data channel signals
  io.tl_out_d_ready := (rf_state === rf_do_ram_read) || (wb_state === wb_wait_ram_bresp)

}


// ============================== DCache ==============================
class TLSimpleL2Cache(bankid: Int, param: TLL2CacheParams)(implicit p: Parameters) extends LazyModule
with HasControlPlaneParameters
{
  val node = TLAdapterNode(
    //clientFn = { c => c.copy(clients = c.clients map { c2 => c2.copy(sourceId = IdRange(0, 1))} )}
  )

  lazy val module = new LazyModuleImp(this) {
    println(s"bankid=$bankid")
    val nWays = p(NL2CacheWays)
    println(s"nWays = $nWays")
    //val nBanks = p(NBanksPerMemChannel)
    val nSets = p(NL2CacheCapacity) * 1024 / 64 / nWays /// nBanks
    println(s"nSets = $nSets")
    val cp = IO(new CPToL2CacheIO().flip())
    (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>
      require(isPow2(nSets))
      require(isPow2(nWays))

      /* parameters */
      val Y = true.B
      val N = false.B

      val blockSize = 64 * 8
      val blockBytes = blockSize / 8
      val innerBeatSize = in.d.bits.params.dataBits
      val innerBeatBytes = innerBeatSize / 8
      val innerDataBeats = blockSize / innerBeatSize
      val outerBeatSize = out.d.bits.params.dataBits
      val outerBeatBytes = outerBeatSize / 8
      val outerDataBeats = blockSize / outerBeatSize
      val split = innerBeatSize / outerBeatSize
      val splitBits = log2Ceil(split)
      val addrWidth = in.a.bits.params.addressBits
      val innerIdWidth = in.a.bits.params.sourceBits
      val outerIdWidth = out.a.bits.params.sourceBits
      val outerBeatLen = log2Ceil(outerBeatBytes)
      val outerBurstLen = outerBeatLen + log2Ceil(outerDataBeats)

      val innerBeatIndexBits = log2Ceil(innerDataBeats)
      val innerBeatBits = log2Ceil(innerBeatBytes)
      val innerBeatLSB = innerBeatBits
      val innerBeatMSB = innerBeatLSB + innerBeatIndexBits - 1

      val in_len_shift = in.a.bits.size >= innerBeatBits.U
      val in_len  = Mux(in_len_shift, ((1.U << in.a.bits.size) >> innerBeatBits) - 1.U, 0.U)  // #word, i.e., arlen in AXI

      // to keep L1 miss & L2 hit penalty small, inner axi bus width should be as large as possible
      // on loongson and zedboard, outer axi bus width are usually 32bit
      // so we require that innerBeatSize to be multiples of outerBeatSize

      val indexBits = log2Ceil(nSets)
      val blockOffsetBits = log2Ceil(blockBytes)
      //val bankBits = log2Ceil(nBanks)
      val tagBits = addrWidth - indexBits - blockOffsetBits //- bankBits
      val offsetLSB = 0
      val offsetMSB = blockOffsetBits - 1
      //val bankLSB = offsetMSB + 1
      //val bankMSB = bankLSB + bankBits - 1
      val indexLSB = offsetMSB + 1 //bank
      val indexMSB = indexLSB + indexBits - 1
      val tagLSB = indexMSB + 1
      val tagMSB = tagLSB + tagBits - 1
      println(s"tag$tagMSB-$tagLSB index$indexMSB-$indexLSB offset$offsetMSB-$offsetLSB")

      val rst_cnt = RegInit(0.asUInt(log2Up(2 * nSets + 1).W))
      val rst = (rst_cnt < UInt(2 * nSets)) && !reset.toBool
      when (rst) { rst_cnt := rst_cnt + 1.U }

      val s_idle :: s_gather_write_data :: s_send_bresp :: s_update_meta :: s_tag_read_req :: s_tag_read_resp :: s_tag_read :: s_merge_put_data :: s_data_read :: s_data_write :: s_wait_ram_awready :: s_do_ram_write :: s_wait_ram_bresp :: s_wait_ram_arready :: s_do_ram_read :: s_data_resp :: Nil = Enum(UInt(), 16)

      val state = Reg(init = s_idle)
      // state transitions for each case
      // read hit: s_idle -> s_tag_read -> s_data_read -> s_data_resp -> s_idle
      // read miss no writeback : s_idle -> s_tag_read -> s_wait_ram_arready -> s_do_ram_read -> s_data_write -> s_update_meta -> s_idle
      // read miss writeback : s_idle -> s_tag_read -> s_data_read -> s_wait_ram_awready -> s_do_ram_write -> s_wait_ram_bresp
      //                              -> s_wait_ram_arready -> s_do_ram_read -> s_data_write -> s_update_meta -> s_idle
      // write hit: s_idle -> s_gather_write_data ->  s_send_bresp -> s_tag_read -> s_data_read -> s_merge_put_data -> s_data_write -> s_update_meta -> s_idle
      // write miss no writeback : s_idle -> s_gather_write_data ->  s_send_bresp -> s_tag_read -> s_wait_ram_arready -> s_do_ram_read
      //                                  -> s_merge_put_data -> s_data_write -> s_update_meta -> s_idle
      // write miss writeback : s_idle -> s_gather_write_data ->  s_send_bresp -> s_tag_read -> s_data_read -> s_wait_ram_awready -> s_do_ram_write -> s_wait_ram_bresp
      //                               -> s_wait_ram_arready -> s_do_ram_read -> s_merge_put_data -> s_data_write -> s_update_meta -> s_idle
      val timer = GTimer()
      val log_prefix = "cycle: %d bankid: %d [L2Cache] state %x "
      def log_raw(prefix: String, fmt: String, tail: String, args: Bits*) = {
        if (param.debug) {
          printf(prefix + fmt + tail, args:_*)
        }
      }

      /** Single log */
      def log(fmt: String, args: Bits*) = log_raw(log_prefix, fmt, "\n", timer +: bankid.U +: state +: args:_*)
      /** Log with line continued */
      def log_part(fmt: String, args: Bits*) = log_raw(log_prefix, fmt, "", timer +: bankid.U +: state +: args:_*)
      /** Log with nothing added */
      def log_plain(fmt: String, args: Bits*) = log_raw("", fmt, "", args:_*)



      when (out.a.fire()) {
        log("out.a.opcode %x, dsid %x, param %x, size %x, source %x, address %x, mask %x, data %x",
          out.a.bits.opcode,
          out.a.bits.dsid,
          out.a.bits.param,
          out.a.bits.size,
          out.a.bits.source,
          out.a.bits.address,
          out.a.bits.mask,
          out.a.bits.data)
      }

      //val in_send_id = in.d.bits.source
      //val in_send_opcode = in.d.bits.opcode

      // state transitions:
      // s_idle: idle state
      // capture requests
      // CheckOneHot(Seq(in.ar.fire(), in.aw.fire(), in.r.fire(), in.w.fire(), in.b.fire()))
      //LYW

      val s1_in = Reg(new L2CacheReq(edgeIn.bundle, dsidWidth))
      val TL2CacheInput = Module(new TLCacheConvertorIn(edgeIn.bundle, dsidWidth))

      val addr = s1_in.address
      val id = s1_in.source
      val opcode = s1_in.opcode
      val dsid = s1_in.dsid
      val ren = s1_in.opcode === TLMessages.Get
      val wen = s1_in.opcode === TLMessages.PutFullData || s1_in.opcode === TLMessages.PutPartialData

      val start_beat = s1_in.address(innerBeatMSB, innerBeatLSB)
      val inner_end_beat_reg = Reg(UInt(4.W))
      val inner_end_beat = Mux(state === s_idle, start_beat + 7.U, inner_end_beat_reg)

      val merge_curr_beat = RegInit(0.asUInt(log2Ceil(8).W))
      val merge_last_beat = merge_curr_beat === inner_end_beat

      TL2CacheInput.io.tl_in_a.bits := in.a.bits
      TL2CacheInput.io.tl_in_a_valid := in.a.valid
      in.a.ready := TL2CacheInput.io.tl_in_a_ready

      in.d.bits := TL2CacheInput.io.tl_in_d.bits
      in.d.valid := TL2CacheInput.io.tl_in_d_valid
      TL2CacheInput.io.tl_in_d_ready := in.d.ready

      val s1_ready = !rst && state === s_idle //Not any pipeline so far!
      val s1_valid = TL2CacheInput.io.cache_s0.valid
      TL2CacheInput.io.cache_rst := rst
      TL2CacheInput.io.cache_s0.ready := s1_ready

      when (s1_valid && s1_ready) {
        s1_in := TL2CacheInput.io.cache_s0.req
        state := s_tag_read_req
        merge_curr_beat := start_beat
        inner_end_beat_reg := start_beat + 7.U
      }

      val s3_valid = Wire(init = Bool(false))
      val s3_ready = TL2CacheInput.io.cache_s3.ready
      when (s1_ready && s1_valid) {
        val wire_opcode = TL2CacheInput.io.cache_s0.req.opcode
        val wire_ren = wire_opcode === TLMessages.Get
        val wire_wen = wire_opcode === TLMessages.PutFullData || wire_opcode === TLMessages.PutPartialData
        log("opcode %x, dsid %x, size %x, source %x, address %x, mask %x, data %x, ren%x/wen%x s1_valid%x s1_ready%x s3_valid%x s3_ready%x",
          TL2CacheInput.io.cache_s0.req.opcode,
          TL2CacheInput.io.cache_s0.req.dsid,
          TL2CacheInput.io.cache_s0.req.size,
          TL2CacheInput.io.cache_s0.req.source,
          TL2CacheInput.io.cache_s0.req.address,
          TL2CacheInput.io.cache_s0.req.mask.asUInt,
          TL2CacheInput.io.cache_s0.req.data.asUInt,
          wire_ren, wire_wen,
          s1_valid,
          s1_ready,
          s3_valid, s3_ready
          )
      }

      // s_tag_read: inspecting meta data
      // to keep the sram access path short, sram addr and output are latched
      // now, tag access has three stages:
      // 1. read req  2. read response  3. check hit, miss

      // metadata array
      val meta_array = DescribedSRAM(
        name = "L2_meta_array",
        desc = "L2 cache metadata array",
        size = nSets,
        data = Vec(nWays, new MetadataEntry(tagBits, dsidWidth))
      )


      val idx = s1_in.address(indexMSB, indexLSB)
      //val bank = addr(bankMSB, bankLSB)

      val meta_array_wen = rst || state === s_tag_read
      val read_tag_req = (state === s_tag_read_req)
      val meta_rdata = meta_array.read(idx, read_tag_req && !meta_array_wen)

      def wayMap[T <: Data](f: Int => T) = Vec((0 until nWays).map(f))

      val vb_rdata = wayMap((w: Int) => meta_rdata(w).valid).asUInt
      val db_rdata = wayMap((w: Int) => meta_rdata(w).dirty).asUInt
      val tag_rdata = wayMap((w: Int) => meta_rdata(w).tag)
      val curr_state = wayMap((w: Int) => meta_rdata(w).rr_state).asUInt
      val set_dsids = wayMap((w: Int) => meta_rdata(w).dsid)

      when (state === s_tag_read_req) {
        state := s_tag_read_resp
      }

      // tag, valid, dirty response
      val vb_rdata_reg = Reg(Bits(width = nWays.W))
      val db_rdata_reg = Reg(Bits(width = nWays.W))
      val tag_rdata_reg = Reg(Vec(nWays, UInt(width = tagBits.W)))
      val curr_state_reg = Reg(Bits(width = nWays))
      val set_dsids_reg = Reg(Vec(nWays, UInt(width = dsidWidth.W)))

      // val set_first_access_flag = RegInit(Vec(Seq.fill(nSets){ Bool(true) }))
      // val init_state = ((1 << nWays) - 1).U(nWays.W)

      when (state === s_tag_read_resp) {
        state := s_tag_read
        vb_rdata_reg := vb_rdata
        db_rdata_reg := db_rdata
        tag_rdata_reg := tag_rdata
        curr_state_reg := curr_state
        set_dsids_reg := set_dsids
      }

      // check hit, miss, repl_way
      val tag = s1_in.address(tagMSB, tagLSB)
      val tag_eq_way = wayMap((w: Int) => tag_rdata_reg(w) === tag)
      val tag_match_way = wayMap((w: Int) => tag_eq_way(w) && vb_rdata_reg(w)).asUInt
      val hit = tag_match_way.orR
      val read_hit = hit && ren
      val write_hit = hit && wen
      val read_miss = !hit && ren
      val write_miss = !hit && wen
      val hit_way = Wire(Bits())
      hit_way := Bits(0)
      (0 until nWays).foreach(i => when (tag_match_way(i)) { hit_way := Bits(i) })


      cp.dsid := s1_in.dsid
      val curr_mask = cp.waymask //0xFFFF.U
      val repl_way = Mux((curr_state_reg & curr_mask).orR, PriorityEncoder(curr_state_reg & curr_mask),
        Mux(curr_mask.orR, PriorityEncoder(curr_mask), UInt(0)))
      val repl_dsid = set_dsids_reg(repl_way)
      val dsid_occupacy = RegInit(Vec(Seq.fill(1 << dsidWidth){ 0.U(log2Ceil(p(NL2CacheCapacity) * 1024 / blockBytes).W) }))
      val requester_occupacy = dsid_occupacy(s1_in.dsid)
      val victim_occupacy = dsid_occupacy(repl_dsid)
      when (state === s_tag_read) {
        log("req_dsid %d occ %d repl_dsid %d occ %d way %d", s1_in.dsid, requester_occupacy, repl_dsid, victim_occupacy, repl_way)
      }

      cp.capacity := dsid_occupacy(cp.capacity_dsid)


      // valid and dirty
      val need_writeback = vb_rdata_reg(repl_way) && db_rdata_reg(repl_way)
      val writeback_tag = tag_rdata_reg(repl_way)
      //val writeback_addr = Cat(writeback_tag, Cat(idx, Cat(bank, 0.U(blockOffsetBits.W))))
      val writeback_addr = Cat(writeback_tag, Cat(idx, 0.U(blockOffsetBits.W)))

      val read_miss_writeback = read_miss && need_writeback
      val read_miss_no_writeback = read_miss && !need_writeback
      val write_miss_writeback = write_miss && need_writeback
      val write_miss_no_writeback = write_miss && !need_writeback

      val need_data_read = read_hit || write_hit || read_miss_writeback || write_miss_writeback

      when (state === s_tag_read) {
        log("hit: %d idx: %x curr_state_reg: %x waymask: %x hit_way: %x repl_way: %x", hit, idx, curr_state_reg, curr_mask, hit_way, repl_way)
        when (ren) {
          log_part("read addr: %x idx: %d tag: %x hit: %d ", addr, idx, tag, hit)
        }
        when (wen) {
          log_part("write addr: %x idx: %d tag: %x hit: %d ", addr, idx, tag, hit)
        }
        when (hit) {
          log_plain("[hit] hit_way: %d\n", hit_way)
        } .elsewhen (need_writeback) {
          log_plain("[wb] repl_way: %d wb_addr: %x\n", repl_way, writeback_addr)
        } .otherwise {
          log_plain("repl_way: %d repl_addr: %x\n", repl_way, writeback_addr)
        }
        log("s1 tags: " + Seq.fill(tag_rdata_reg.size)("%x").mkString(" "), tag_rdata_reg:_*)
        log("s1 vb: %x db: %x", vb_rdata_reg, db_rdata_reg)

        // check for cross cache line bursts
        assert(inner_end_beat < innerDataBeats.U, s"cross cache line bursts detected  inner_end_beat$inner_end_beat < innerDataBeats${innerDataBeats.U} addr$addr")

        when (read_hit || write_hit || read_miss_writeback || write_miss_writeback) {
          state := s_data_read
        } .elsewhen (read_miss_no_writeback || write_miss_no_writeback) {
          // no need to write back, directly refill data
          state := s_wait_ram_arready
        } .otherwise {
          assert(N, s"Unexpected condition in s_tag_read read_hit$read_hit write_hit$write_hit state$state")
        }
      }

      val rst_metadata = Wire(Vec(nWays, new MetadataEntry(tagBits, 16)))
      for (i <- 0 until nWays) {
        val metadata = rst_metadata(i)
        metadata.valid := false.B
        metadata.dirty := false.B
        metadata.tag := 0.U
        metadata.dsid := 0.U
      }


      // update metadata

      val update_way = Mux(hit, hit_way, repl_way)
      val next_state = Wire(Bits())
      when (state === s_tag_read) {
        when (!(curr_state_reg & curr_mask).orR) {
          next_state := curr_state_reg | curr_mask
        } .otherwise {
          next_state := curr_state_reg.bitSet(update_way, Bool(false))
        }
        log("dsid: %d set: %d hit: %d rw: %d update_way: %d curr_state: %x next_state: %x",
          s1_in.dsid, idx, hit, ren, update_way, curr_state_reg, next_state)
      }

      val update_metadata = Wire(Vec(nWays, new MetadataEntry(tagBits, 16)))
      for (i <- 0 until nWays) {
        val metadata = update_metadata(i)
        val is_update_way = update_way === i.U
        when (is_update_way) {
          metadata.valid := true.B
          metadata.dirty := Mux(read_hit, db_rdata_reg(update_way),
            Mux(read_miss, false.B, true.B))
          metadata.tag := tag
          metadata.dsid := s1_in.dsid
        } .otherwise {
          metadata.valid := vb_rdata_reg(i)
          metadata.dirty := db_rdata_reg(i)
          metadata.tag := tag_rdata_reg(i)
          metadata.dsid := set_dsids_reg(i)
        }
        metadata.rr_state := next_state(i)
      }

      when (state === s_tag_read) {
        val fmt_part = Seq.tabulate(dsid_occupacy.size) { _ + ": %d" }.mkString(", ")
        log("dsid_occ = " + fmt_part, dsid_occupacy: _*)
      }

      val meta_array_widx = Mux(rst, rst_cnt, idx)
      val meta_array_wdata = Mux(rst, rst_metadata, update_metadata)

      when (meta_array_wen) {
        meta_array.write(meta_array_widx, meta_array_wdata)
        // Update dsid occupacy stat
        when (!hit && !rst) {
          assert(update_way === repl_way, "update must = repl way when decrease a dsid's occupacy")
          val victim_valid = vb_rdata_reg(repl_way)
          dsid_occupacy.zipWithIndex foreach { case (dsid_occ, i) =>
              when (i.U === s1_in.dsid && (!victim_valid || i.U =/= repl_dsid)) {
                dsid_occ := requester_occupacy + 1.U
              }.elsewhen(i.U =/= s1_in.dsid && i.U === repl_dsid && victim_valid) {
                dsid_occ := victim_occupacy - 1.U
              }
          }
          when (victim_valid) {
            log("victim dsid %d dec way %d old_value %d", repl_dsid, repl_way, victim_occupacy)
          }
          log("dsid %d inc way %d old_value %d", s1_in.dsid, update_way, requester_occupacy)
        }
      }

      // ###############################################################
      // #                  data array read/write                      #
      // ###############################################################
      val data_read_way = Mux(read_hit || write_hit, hit_way, repl_way)
      // incase it overflows
      val data_read_cnt = RegInit(0.asUInt((log2Ceil(innerDataBeats) + 1).W))
      val data_read_valid = (state === s_tag_read && need_data_read) || (state === s_data_read && data_read_cnt =/= innerDataBeats.U)
      val data_read_idx = idx << log2Ceil(innerDataBeats) | data_read_cnt
      val dout = Wire(Vec(split, UInt(outerBeatSize.W)))

      val data_write_way = Mux(write_hit, hit_way, repl_way)
      val data_write_cnt = Wire(UInt())
      val data_write_valid = state === s_data_write
      val data_write_idx = idx << log2Ceil(innerDataBeats) | data_write_cnt
      val din = Wire(Vec(split, UInt(outerBeatSize.W)))

      val data_arrays = Seq.fill(split) {
        DescribedSRAM(
          name = "L2_data_array",
          desc = "L2 data array",
          size = nSets * innerDataBeats,
          data = Vec(nWays, UInt(width = outerBeatSize.W))
        ) }
      for ((data_array, i) <- data_arrays zipWithIndex) {
        when (data_write_valid) {
          log("write data array: %d idx: %d way: %d data: %x", i.U, data_write_idx, data_write_way, din(i))
          data_array.write(data_write_idx, Vec.fill(nWays)(din(i)), (0 until nWays).map(data_write_way === UInt(_)))
        }
        dout(i) := data_array.read(data_read_idx, data_read_valid && !data_write_valid)(data_read_way)
        when (RegNext(data_read_valid, N)) {
          log("read data array: %d idx: %d way: %d data: %x",
            i.U, RegNext(data_read_idx), RegNext(data_read_way), dout(i))
        }
      }

      // s_data_read
      // read miss or need write back
      val data_buf = Reg(Vec(outerDataBeats, UInt(outerBeatSize.W)))
      when (data_read_valid) {
        data_read_cnt := data_read_cnt + 1.U
      }
      when (state === s_data_read) {
        for (i <- 0 until split) {
          data_buf(((data_read_cnt - 1.U) << splitBits) + i.U) := dout(i)
        }
        when (data_read_cnt === innerDataBeats.U) {
          data_read_cnt := 0.U
          when (read_hit) {
            state := s_data_resp
          } .elsewhen (read_miss_writeback || write_miss_writeback) {
            state := s_wait_ram_awready
          } .elsewhen (write_hit) {
            state := s_merge_put_data
          } .otherwise {
            assert(N, "Unexpected condition in s_data_read")
          }
        }
      }

      // s_merge_put_data: merge data_buf and put_data_buf, and store the final result in data_buf
      // the old data(either read from data array or from refill) resides in data_buf
      // the new data(gathered from inner write) resides in put_data_buf
      
      def mergePutData(old_data: UInt, new_data: UInt, wmask: UInt): UInt = {
        val full_wmask = FillInterleaved(8, wmask)
        ((~full_wmask & old_data) | (full_wmask & new_data))
      }
      when (state === s_merge_put_data) {
        merge_curr_beat := merge_curr_beat + 1.U
        val idx = merge_curr_beat
        data_buf(idx) := mergePutData(data_buf(idx), s1_in.data(idx), s1_in.mask(idx))
        when (merge_last_beat) {
          state := s_data_write
        }
      }

      // s_data_write
      val (write_cnt, write_done) = Counter(state === s_data_write, innerDataBeats)
      data_write_cnt := write_cnt
      din(data_write_cnt) := data_buf(data_write_cnt)
      
      when (state === s_data_write && write_done) {
        when (ren) {
          state := s_data_resp
        } .otherwise {
          state := s_idle
        }
      }

      // outer tilelink interface
      // external memory bus width is 32/64/128bits
      // so each memport read/write is mapped into a whole tilelink bus width read/write
      //val axi4_size = log2Up(outerBeatBytes).U
      //val mem_addr = Cat(addr(tagMSB, bankLSB), 0.asUInt(blockOffsetBits.W))
      val mem_addr = Cat(s1_in.address(tagMSB, indexLSB), 0.asUInt(blockOffsetBits.W))

      val out_raddr_fire = out.a.fire()
      val out_waddr_fire = out.a.fire()
      val out_wdata_fire = out.a.fire()
      val out_wreq_fire = out.d.fire()
      val out_rdata_fire = out.d.fire()
      val out_rdata = out.d.bits.data
      // #########################################################
      // #                  write back path                      #
      // #########################################################
      // s_wait_ram_awready

      val TL2CacheOutput = Module(new TLCacheConvertorOut(edgeIn.bundle, dsidWidth))
      TL2CacheOutput.io.dsid := s1_in.dsid
      out.a.bits := TL2CacheOutput.io.tl_out_a.bits
      out.a.valid := TL2CacheOutput.io.tl_out_a_valid
      TL2CacheOutput.io.tl_out_a_ready := out.a.ready
      TL2CacheOutput.io.tl_out_d.bits := out.d.bits
      TL2CacheOutput.io.tl_out_d_valid := out.d.valid
      out.d.ready := TL2CacheOutput.io.tl_out_d_ready

      TL2CacheOutput.io.wb_addr := writeback_addr
      TL2CacheOutput.io.wb_data := data_buf
      TL2CacheOutput.io.wb_valid := state === s_wait_ram_awready
      when (TL2CacheOutput.io.wb_ready && state === s_wait_ram_awready) {
        state := s_wait_ram_arready // also wb_valid -> false
      }


      // when (state === s_wait_ram_awready) {
      //   state := s_do_ram_write
      // }

      // val (wb_cnt, wb_done) = Counter(out_wdata_fire && state === s_do_ram_write, outerDataBeats)
      // when (state === s_do_ram_write && wb_done) {
      //   state := s_wait_ram_bresp
      // }

      // val out_write_valid = state === s_do_ram_write

      // when (state === s_wait_ram_bresp && out_wreq_fire) {
      //   when (read_miss_writeback || write_miss_writeback) {
      //     // do refill
      //     state := s_wait_ram_arready
      //   } .otherwise {
      //     assert(N, "Unexpected condition in s_wait_ram_bresp")
      //   }
      // }

      // #####################################################
      // #                  refill path                      #
      // #####################################################
      TL2CacheOutput.io.rf_addr := mem_addr
      TL2CacheOutput.io.rf_addr_valid := state === s_wait_ram_arready
      TL2CacheOutput.io.rf_ready := state === s_wait_ram_arready
      when (TL2CacheOutput.io.rf_ready && TL2CacheOutput.io.rf_data_valid) {
        data_buf := TL2CacheOutput.io.rf_data
        when (ren) {
          state := s_data_write // also rf_ready turns false
        } .otherwise {
          state := s_merge_put_data
        }
      }

      // when (state === s_wait_ram_arready && out_raddr_fire) {
      //   state := s_do_ram_read
      // }
      // val (refill_cnt, refill_done) = Counter(out_rdata_fire && state === s_do_ram_read, outerDataBeats)
      // when (state === s_do_ram_read && out_rdata_fire) {
      //   data_buf(refill_cnt) := out_rdata
      //   when (refill_done) {
      //     when (ren) {
      //       state := s_data_write
      //     } .otherwise {
      //       state := s_merge_put_data
      //     }
      //   }
      // }

      // val out_read_valid = state === s_wait_ram_arready

      // val out_data = data_buf(wb_cnt)
      // val out_addr = Mux(out_read_valid, mem_addr, writeback_addr)
      // val out_opcode = Mux(out_read_valid, TLMessages.Get, TLMessages.PutFullData)

      // out.a.bits.opcode  := out_opcode
      // out.a.bits.dsid    := s1_in.dsid
      // out.a.bits.param   := UInt(0)
      // out.a.bits.size    := outerBurstLen.U
      // out.a.bits.source  := 0.asUInt(outerIdWidth.W)
      // out.a.bits.address := out_addr
      // out.a.bits.mask    := edgeOut.mask(out_addr, outerBurstLen.U)
      // //printf("[debug] mask %x \n", out.a.bits.mask)

      // out.a.bits.data    := out_data
      // out.a.bits.corrupt := Bool(false)
      // //edgeOut.Put(0.asUInt(outerIdWidth.W), out_addr, outerBeatLen.U, out_data)
      
      // out.a.valid := out_write_valid || out_read_valid

      // // read data channel signals
      // out.d.ready := (state === s_do_ram_read) || (state === s_wait_ram_bresp)

      // ########################################################
      // #                  data resp path                      #
      // ########################################################
      //val cache_s3 = Reg(new L2CacheReq(edgeIn.bundle, dsidWidth))
      
      //cache_s3 := s1_in //because there is no real pipeline so far
      TL2CacheInput.io.cache_s3.req := s1_in //cache_s3
      TL2CacheInput.io.cache_s3.req.data := data_buf
      TL2CacheInput.io.cache_s3.valid := s3_valid

      when (state === s_data_resp) {
        s3_valid := Bool(true)
        when (s3_ready) {
          state := s_idle
        }
      }

      if (edgeOut.manager.anySupportAcquireB && edgeOut.client.anySupportProbe) {
        in.b <> out.b
        out.c <> in.c
        out.e <> in.e
      } else {
        in.b.valid := Bool(false)
        in.c.ready := Bool(true)
        in.e.ready := Bool(true)
        out.b.ready := Bool(true)
        out.c.valid := Bool(false)
        out.e.valid := Bool(false)
      }
    }
  }
}

object TLSimpleL2Cache
{
  def apply(bankid: Int)(implicit p: Parameters): TLNode =
  {
    if (p(NL2CacheCapacity) != 0) {
      val tlsimpleL2cache = LazyModule(new TLSimpleL2Cache(bankid, TLL2CacheParams()))
      tlsimpleL2cache.node
    }
    else {
      val tlsimpleL2cache = LazyModule(new TLBuffer(BufferParams.none))
      tlsimpleL2cache.node
    }
  }
}

object TLSimpleL2CacheRef
{
  def apply(bankid: Int)(implicit p: Parameters): TLSimpleL2Cache = {
    val tlsimpleL2cache = LazyModule(new TLSimpleL2Cache(bankid, TLL2CacheParams()))
    tlsimpleL2cache
  }
}
