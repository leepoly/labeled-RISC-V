// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.util

import Chisel._

abstract class ReplacementPolicy {
  def way: UInt
  def miss: Unit
  def hit: Unit
}

class RandomReplacement(ways: Int) extends ReplacementPolicy {
  private val replace = Wire(Bool())
  replace := Bool(false)
  val lfsr = LFSR16(replace)

  def way = if(ways == 1) UInt(0) else lfsr(log2Up(ways)-1,0)
  def miss = replace := Bool(true)
  def hit = {}
}

abstract class SeqReplacementPolicy {
  def name: String
  def access(set: UInt): Unit
  def update(valid: Bool, hit: Bool, set: UInt, way: UInt, core: UInt): Unit
  def way: UInt
}

class SeqRandom(n_ways: Int) extends SeqReplacementPolicy {
  val logic = new RandomReplacement(n_ways)
  def name = "Random"
  def access(set: UInt) = { }
  def update(valid: Bool, hit: Bool, set: UInt, way: UInt, core: UInt) = {
    when (valid && !hit) { logic.miss }
  }
  def way = logic.way
}

class PseudoLRU(n: Int)
{
  private val state_reg = Reg(UInt(width = n-1))
  def access(way: UInt) {
    state_reg := get_next_state(state_reg,way)
  }
  def get_next_state(state: UInt, way: UInt) = {
    var next_state = state << 1
    var idx = UInt(1,1)
    for (i <- log2Up(n)-1 to 0 by -1) {
      val bit = way(i)
      next_state = next_state.bitSet(idx, !bit)
      idx = Cat(idx, bit)
    }
    next_state.extract(n-1, 1)
  }
  def replace = get_replace_way(state_reg)
  def get_replace_way(state: UInt) = {
    val shifted_state = state << 1
    var idx = UInt(1,1)
    for (i <- log2Up(n)-1 to 0 by -1) {
      val in_bounds = Cat(idx, UInt(BigInt(1) << i))(log2Up(n)-1, 0) < UInt(n)
      idx = Cat(idx, in_bounds && shifted_state(idx))
    }
    idx(log2Up(n)-1,0)
  }
}

class SeqPLRU(n_sets: Int, n_ways: Int) extends SeqReplacementPolicy {
  val state = SeqMem(n_sets, UInt(width = n_ways-1))
  val logic = new PseudoLRU(n_ways)
  val current_state = Wire(UInt())
  val plru_way = logic.get_replace_way(current_state)
  val next_state = Wire(UInt())

  def name = "PLRU"

  def access(set: UInt) = {
    current_state := state.read(set)
  }

  def update(valid: Bool, hit: Bool, set: UInt, way: UInt, core: UInt) = {
    val update_way = Mux(hit, way, plru_way)
    next_state := logic.get_next_state(current_state, update_way)
    when (valid) { state.write(set, next_state) }
  }

  def way = plru_way
}

//DRRIP
class DRRIP(n_cores: Int, n_sets: Int, n_ways: Int,
            psel_width: Int, sdm_size: Int) {

  private val N_CORE_BITS = Math.log(n_cores).toInt
  private val N_SDM_BITS = Math.log(sdm_size).toInt
  private val N_SET_BITS = Math.log(n_sets).toInt
  private val N_STATE_BITS = 2
  private val N_STATE_COUNT = Math.pow(N_STATE_BITS, 2).toInt
  private val STATE_COUNT_MAX = N_STATE_COUNT - 1
  private val BIP_MAX = 31
  private val PSEL_MAX = (1 << psel_width) - 1
  private val PSEL_THRS = PSEL_MAX / 2


  val state_table = RegInit(Vec(Seq.fill(n_sets)(UInt(0, width = n_ways * 2))))
  val psel_table = RegInit(Vec(Seq.fill(n_cores)(UInt(0, width = psel_width))))

  val psel_counter_value = Wire(UInt(psel_width.W))
  val bip_counter_next = Wire(UInt(6.W))
  val bip_counter = RegInit(0.U(6.W))

  val next_state = Wire(UInt((n_ways * 2).W))
  val current_state = Wire(UInt((n_ways * 2).W))
  val compensated_state = Wire(Vec(n_ways, UInt(2.W)))

  val found_vec = Wire(Vec(N_STATE_COUNT, UInt(n_ways.W)))
  val first_vec = Wire(Vec(N_STATE_COUNT, UInt(n_ways.W)))

  val drrip_way = Wire(UInt(32.W))

  val state_mask = Wire(Vec(n_ways, UInt(2.W)))
  val way_state_next = Wire(Vec(n_ways, UInt(2.W)))


  def name = "DRRIP"

  def access(set: UInt) = {

    val current_access_state = Wire(UInt((n_ways * 2).W))

    val found_0 = Wire(Vec(n_ways, Bool()))
    val found_1 = Wire(Vec(n_ways, Bool()))
    val found_2 = Wire(Vec(n_ways, Bool()))
    val found_3 = Wire(Vec(n_ways, Bool()))

    val first_0 = Wire(UInt(n_ways.W))
    val first_1 = Wire(UInt(n_ways.W))
    val first_2 = Wire(UInt(n_ways.W))
    val first_3 = Wire(UInt(n_ways.W))

    //val state_add = Wire(UInt(1.W))
    //val source_add = Wire(UInt(n_ways.W))

    current_access_state := state_table(set)

    //found_vec.zipWithIndex.foreach((found_with_index) => {found_with_index._1 := (current_state(found_with_index._2*2+1, found_with_index._2*2) === found_with_index._2.U)})


    for (i <- n_ways - 1 to 0 by -1) {

      found_0(i) := (~(current_access_state(i*2+1) | current_access_state(i*2))).toBool()
      found_1(i) := ((~current_access_state(i*2+1)).asUInt() & (current_access_state(i*2))).toBool()
      found_2(i) := (current_access_state(i*2+1) & ((~current_access_state(i*2)).asUInt())).toBool()
      found_3(i) := (current_access_state(i*2+1) & (current_access_state(i*2))).toBool()
      //compensated_state(n_ways - 1 - i) := Mux(state_add.toBool(),
      //                                            (current_state(i*2+1, i*2) + source_add)(1, 0), current_state(i*2+1, i*2))
    }

    //state_add := ~(found_vec.last.orR())
    //source_add := PriorityEncoder(found_vec.map((c) => {c.orR()}))
    //first_vec.zipWithIndex.map((c) => {if (c._2 == 0) {c._1 := 0.U} else {c._1 := PriorityEncoder(found_vec(c._2))}})
    //drrip_way := first_vec(source_add)

    //state_add := ~(found_3.reduce(_|_))
    //source_add := Mux(found_2.reduce(_|_), 1.U, Mux(found_1.reduce(_|_), 2.U, 3.U))

    first_0 := 0.U
    first_1 := PriorityEncoder(found_1.reverse)
    first_2 := PriorityEncoder(found_2.reverse)
    first_3 := PriorityEncoder(found_3.reverse)

    drrip_way := Mux(found_3.reduce(_|_), first_3, Mux(found_2.reduce(_|_), first_2, Mux(found_1.reduce(_|_), first_1, first_0)))
  }


  def update(valid: Bool, hit: Bool, set: UInt, way: UInt, cpu: UInt) = {

    current_state := Mux(valid, state_table(set), 0.U)

    val core_bits = Wire(UInt(N_CORE_BITS.W))
    val policy_bit = Wire(UInt(1.W))
    val sdm_bits = Wire(UInt(N_SDM_BITS.W))

    val new_way_block = Wire(UInt(2.W))
    val miss_state = Wire(UInt((n_ways * 2).W))
    val hit_state = compensated_state.reduce(Cat(_, _)) & state_mask.reduce(Cat(_, _))

    val update_found_0 = Wire(Vec(n_ways, Bool()))
    val update_found_1 = Wire(Vec(n_ways, Bool()))
    val update_found_2 = Wire(Vec(n_ways, Bool()))
    val update_found_3 = Wire(Vec(n_ways, Bool()))

    val state_add = Wire(UInt(1.W))
    val source_add = Wire(UInt(n_ways.W))

    core_bits   := set(N_SET_BITS - 1, N_SET_BITS - N_CORE_BITS)
    policy_bit  := set(N_SET_BITS - N_CORE_BITS - 1)
    sdm_bits    := set(N_SDM_BITS - 1 , 0)

    next_state := Mux(valid, Mux(hit, hit_state, miss_state), 0.U)
    bip_counter_next := bip_counter + 1.U
    psel_counter_value := psel_table(cpu)

    // current compensated_state

    for (i <- n_ways - 1 to 0 by -1) {

      update_found_0(i) := (~(current_state(i*2+1) | current_state(i*2))).toBool()
      update_found_1(i) := ((~current_state(i*2+1)).asUInt() & (current_state(i*2))).toBool()
      update_found_2(i) := (current_state(i*2+1) & ((~current_state(i*2)).asUInt())).toBool()
      update_found_3(i) := (current_state(i*2+1) & (current_state(i*2))).toBool()
      compensated_state(n_ways - 1 - i) := Mux(state_add.toBool(),
                                                  (current_state(i*2+1, i*2) + source_add)(1, 0), current_state(i*2+1, i*2))
    }

    state_add := ~(update_found_3.reduce(_|_))
    source_add := Mux(update_found_2.reduce(_|_), 1.U, Mux(update_found_1.reduce(_|_), 2.U, 3.U))


    for (i <- 0 until n_ways) {
      state_mask(i) := Mux(way === i.U, 0.U, (N_STATE_COUNT - 1).U)
      way_state_next(i) := Mux(way === i.U, new_way_block, 0.U)
    }
    miss_state := compensated_state.reduce(Cat(_, _)) & state_mask.reduce(Cat(_, _)) | way_state_next.reduce(Cat(_, _))


    when ((~hit).toBool() & valid) {
      // SDM
      val not_update_psel = (sdm_bits.orR()).toBool() & (core_bits === cpu)
      val lead_to_bip = Mux(not_update_psel, psel_counter_value > PSEL_THRS.U, ~policy_bit).toBool()
      val psel_counter_next = Mux(not_update_psel, psel_counter_value,
                                Mux(lead_to_bip, psel_counter_value - 1.U, psel_counter_value + 1.U))

      when(lead_to_bip) {
        // BIP
        psel_table(cpu) := psel_counter_next
        bip_counter := bip_counter_next
        new_way_block := Mux(bip_counter === BIP_MAX.U, (PSEL_MAX - 1).U, PSEL_MAX.U)
      }.otherwise {
        // SRRIP
        psel_table(cpu) := psel_counter_next
        new_way_block := (PSEL_MAX - 1).U
      }
    }.otherwise {
      new_way_block := 0.U
    }

    when (valid) {
        state_table(set) := next_state
    }
  }

  def way = drrip_way
}

// class SRRIPLogic(n_state_bits: Int, n_ways: Int) {
//
//     private val N_STATE_COUNT = Math.pow(n_state_bits, 2).toInt
//
//     val state_reg = Reg(UInt(width = n-1))
//     val found_vec = Seq(N_STATE_COUNT, Wire(UInt(n_ways.W)))
//     val first_vec = Seq(N_STATE_COUNT, Wire(UInt(n_ways.W)))
//     val exist_vec = Seq(N_STATE_COUNT, Wire(UInt(1.W)))
//
//     // Little-End
//     def split(state: UInt) {
//         val staeSplited = Seq(n_ways, UInt(n_state_bits.W))
//         for (i <- 0 until n_ways) {
//             staeSplited(i) := state(i*2+1, i*2)
//         }
//     }
//
//     // Little-End
//     def merge(state: Seq(UInt)) {
//         state.reduce((a, b) => Cat(b, a))
//     }
//
//     def access(way: UInt) {
//       state_reg := get_next_state(state_reg,way)
//     }
//     def get_next_state(hit: Bool, state: UInt, way: UInt) = {
//         val hit_state = Wire(UInt((n_ways * n_state_bits).W))
//         val miss_state = Wire(UInt((n_ways * n_state_bits).W))
//         val next_state = Mux(hit, hit_state, miss_state)
//         val final_state = Wire(UInt((n_ways * n_state_bits).W))
//
//         val state_mask = Wire(UInt((n_ways * n_state_bits).W))
//         val compensated_state = Wire(UInt((n_ways * n_state_bits).W))
//         state_mask(i) := Mux(way === i.U, 0.U, (N_STATE_COUNT - 1).U)
//
//         state_mask
//
//         final_state := state_mask & compensated_state | next_state
//     }
//     def replace = get_replace_way(state_reg)
//     def get_replace_way(state: UInt) = {
//       val stateSplited = split(state)
//       val repla_way = Wire(UInt(n_ways.W))
//
//       found_vec.zipWithIndex.map((found, index) => {
//           found := stateSplited.map((state_bits) => {
//               state_bits === index.U
//           })
//       })
//
//       first_vec.map((first) => {
//           first := PriorityEncoder(found_vec(index).reverse)
//       })
//
//       exist_vec.zipWithIndex.map((exist, index) => {
//           exist := found_vec(index).orR
//       })
//
//       repla_way := first_vec(PriorityEncoder(merge(exist)))
//     }
// }
//
// class SRRIP(n_sets: Int, n_ways: Int) extends SeqReplacementPolicy {
//
//   private val N_STATE_BITS = 2
//   private val N_STATE_COUNT = Math.pow(N_STATE_BITS, 2).toInt
//
//   val state = SeqMem(n_sets, UInt(width = n_ways * N_STATE_BITS))
//   val logic = new SRRIPLogic(n_ways)
//   val current_state = Wire(UInt())
//   val srrip_way = logic.get_replace_way(current_state)
//   val next_state = Wire(UInt())
//
//   def name = "SRRIP"
//
//   def access(set: UInt) = {
//     current_state := state.read(set)
//   }
//
//   def update(valid: Bool, hit: Bool, set: UInt, way: UInt) = {
//     val update_way = Mux(hit, way, srrip_way)
//     next_state := logic.get_next_state(hit, current_state, update_way)
//     when (valid) { state.write(set, next_state) }
//   }
//
//   def way = srrip_way
// }

// class SHiPLogic()
//
// class SeqSHiP(n_sets: UInt, n_ways: UInt) extends SeqReplacementPolicy {
//     val
// }
