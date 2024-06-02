/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/
package xiangshan.frontend

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import utility._
import xiangshan._
import xiangshan.frontend._
// 定义了一个名为RAS项，它包含两个字段：retAddr（返回地址）和ctr（嵌套调用函数的层级）。此外，定义了一个=/=方法，用于比较两个RASEntry实例是否不同。
class RASEntry()(implicit p: Parameters) extends XSBundle {
    val retAddr = UInt(VAddrBits.W)
    val ctr = UInt(8.W) // 函数嵌套层数
    def =/=(that: RASEntry) = this.retAddr =/= that.retAddr || this.ctr =/= that.ctr
}
// RASPtr类是一个循环队列指针，用于管理RAS中的条目。它包含两个字段：flag（标志位）和value（值）。flag用于标识指针是否有效，value用于存储指针的值。RASPtr类还定义了一个apply方法，用于创建RASPtr实例，以及一个inverse方法，用于创建一个与指定RASPtr实例相反的实例。
class RASPtr(implicit p: Parameters) extends CircuularQueuePtr[RASPtr](
  p => p(XSCoreParamsKey).RasSpecSize
){
}

// 赋值
object RASPtr {
  def apply(f: Bool, v: UInt)(implicit p: Parameters): RASPtr = {
    val ptr = Wire(new RASPtr)
    ptr.flag := f
    ptr.value := v
    ptr
  }
  // 状态翻转
  def inverse(ptr: RASPtr)(implicit p: Parameters): RASPtr = {
    apply(!ptr.flag, ptr.value)
  }
}


class RASMeta(implicit p: Parameters) extends XSBundle {
val ssp = UInt(log2Up(RasSize).W) // 当前的栈指针
  val sctr = UInt(RasCtrSize.W) // 追踪嵌套的层级
  val TOSW = new RASPtr // RAS预测栈写指针
  val TOSR = new RASPtr // RAS预测栈读指针
  val NOS = new RASPtr  // RAS栈顶指针
}

object RASMeta {
  def apply(ssp: UInt, sctr: UInt, TOSW: RASPtr, TOSR: RASPtr, NOS: RASPtr)(implicit p: Parameters):RASMeta = {
    val e = Wire(new RASMeta)
    e.ssp := ssp
    e.sctr := sctr
    e.TOSW := TOSW
    e.TOSR := TOSR
    e.NOS := NOS
    e
  }
}

class RASDebug(implicit p: Parameters) extends XSBundle { // 用于调试的RASDebug类，包含两个字段：commit_stack（提交栈）和spec_nos（预测栈指针）。commit_stack用于存储提交栈的条目，spec_nos用于存储预测栈指针。
  val spec_queue = Output(Vec(RasSpecSize, new RASEntry))
  val spec_nos = Output(Vec(RasSpecSize, new RASPtr))
  val commit_stack = Output(Vec(RasSize, new RASEntry))
}

class RAS(implicit p: Parameters) extends BasePredictor { // RAS 类继承自 BasePredictor 类，用于实现函数调用的返回地址栈（Return Address Stack，RAS）。
  override val meta_size = WireInit(0.U.asTypeOf(new RASMeta)).getWidth

  object RASEntry {
    def apply(retAddr: UInt, ctr: UInt): RASEntry = {
      val e = Wire(new RASEntry)
      e.retAddr := retAddr
      e.ctr := ctr
      e
    }
  }


  class RASStack(rasSize: Int, rasSpecSize: Int) extends XSModule with HasCircularQueuePtrHelper {
    val io = IO(new Bundle {
      val spec_push_valid = Input(Bool()) //  进行PUSH操作
      val spec_pop_valid = Input(Bool()) //  进行POP操作
      val spec_push_addr = Input(UInt(VAddrBits.W)) //  PUSH的地址
      // for write bypass between s2 and s3

      val s2_fire = Input(Bool()) // s2阶段是否有效
      val s3_fire = Input(Bool()) // s3阶段是否有效
      val s3_cancel = Input(Bool()) // S3的信号表示需要撤销S2的操作
      val s3_meta = Input(new RASMeta)  // S3需要S2时的现场信息
      val s3_missed_pop = Input(Bool()) // S3是否需要再次POP
      val s3_missed_push = Input(Bool())  // S3是否需要再次PUSH
      val s3_pushAddr = Input(UInt(VAddrBits.W))  // s3 PUSH 的地址
      val spec_pop_addr = Output(UInt(VAddrBits.W)) // POP地址

      val commit_push_valid = Input(Bool()) // 提交栈push有效
      val commit_pop_valid = Input(Bool())  // 提交栈pop有效
      val commit_push_addr = Input(UInt(VAddrBits.W)) // 提交栈的push地址，即返回地址
      val commit_meta_TOSW = Input(new RASPtr)  // 之前预测时候的现场信息TOSW
      val commit_meta_TOSR = Input(new RASPtr)  // 之前预测时候的现场信息TOSR

      // Debug的时候输出的信号
      // for debug purpose only

      val commit_meta_ssp = Input(UInt(log2Up(RasSize).W))  // 提交栈的栈指针
      val commit_meta_sctr = Input(UInt(RasCtrSize.W))  // 提交栈的栈计数器

      val redirect_valid = Input(Bool())  // 重定向是否有效
      val redirect_isCall = Input(Bool()) // 重定向是否是CALL
      val redirect_isRet = Input(Bool())  // 重定向是否是RET
      val redirect_meta_ssp = Input(UInt(log2Up(RasSize).W))  // 重定向时的现场信息ssp
      val redirect_meta_sctr = Input(UInt(RasCtrSize.W))  // 重定向时的现场信息sctr
      val redirect_meta_TOSW = Input(new RASPtr)  // 重定向时的现场信息TOSW
      val redirect_meta_TOSR = Input(new RASPtr)  // 重定向时的现场信息TOSR
      val redirect_meta_NOS = Input(new RASPtr) // 重定向时的现场信息NOS
      val redirect_callAddr = Input(UInt(VAddrBits.W))  // 重定向时的CALL地址

      val ssp = Output(UInt(log2Up(RasSize).W)) // 栈指针
      val sctr = Output(UInt(RasCtrSize.W)) // 追踪嵌套的层级
      val nsp = Output(UInt(log2Up(RasSize).W)) // commit栈顶，会被ssp覆盖
      val TOSR = Output(new RASPtr) // RAS 预测栈 栈顶指针
      val TOSW = Output(new RASPtr) // RAS 预测栈 分配内存指针
      val NOS = Output(new RASPtr)  // RAS栈顶指针
      val BOS = Output(new RASPtr)  // RAS 预测栈 栈底指针

      val debug = new RASDebug
    })

    val commit_stack = RegInit(VecInit(Seq.fill(RasSize)(RASEntry(0.U, 0.U))))  // 提交栈
    val spec_queue = RegInit(VecInit(Seq.fill(rasSpecSize)(RASEntry(0.U, 0.U))))  // 预测栈
    val spec_nos = RegInit(VecInit(Seq.fill(rasSpecSize)(RASPtr(false.B, 0.U))))  // 预测栈指针

    val nsp = RegInit(0.U(log2Up(rasSize).W)) // commit栈顶，会被ssp覆盖
    val ssp = RegInit(0.U(log2Up(rasSize).W)) // commit栈顶指针

    val sctr = RegInit(0.U(RasCtrSize.W)) // RAS预测栈栈顶递归计数 Counter
    val TOSR = RegInit(RASPtr(true.B, (RasSpecSize - 1).U)) // RAS 预测栈读指针
    val TOSW = RegInit(RASPtr(false.B, 0.U))  // RAS 预测栈写指针
    val BOS = RegInit(RASPtr(false.B, 0.U)) // 循环数组起始标记

    val spec_overflowed = RegInit(false.B)  // 标记预测栈是否溢出

    val writeBypassEntry = Reg(new RASEntry)  // 写回绕过机制
    val writeBypassNos = Reg(new RASPtr)  // 写回绕过指针

    val writeBypassValid = RegInit(0.B) // 旁路有效
    val writeBypassValidWire = Wire(Bool()) // 旁路有效线性？

    /* 在实际的硬件设计中，writeBypassValid 可能与 writeBypassValidWire 相关联，
    后者可能是前者的驱动信号之一。例如，writeBypassValidWire 可以根据当前周期的输入
    条件计算出一个值，然后这个值可以在时钟的下一个上升沿被用来更新 writeBypassValid 寄存器。
    这样的设计允许硬件在保持状态的同时，也能够响应即时的逻辑变化。*/

    def TOSRinRange(currentTOSR: RASPtr, currentTOSW: RASPtr) = { // 判断TOSR是否在范围内
      val inflightValid = WireInit(false.B)
      // if in range, TOSR should be no younger than BOS and strictly younger than TOSW
      when (!isBefore(currentTOSR, BOS) && isBefore(currentTOSR, currentTOSW)) {
        inflightValid := true.B
      }
      inflightValid
    }

    def getCommitTop(currentSsp: UInt) = {  // 获取提交栈顶
      commit_stack(currentSsp)
    }

    def getTopNos(currentTOSR: RASPtr, allowBypass: Boolean):RASPtr = { // 获取栈顶指针
      val ret = Wire(new RASPtr)
      if (allowBypass){
        when (writeBypassValid) {
          ret := writeBypassNos
        } .otherwise {
          ret := spec_nos(TOSR.value)
        }
      } else {
        ret := spec_nos(TOSR.value) // invalid when TOSR is not in range
      }
      ret
    }
    // 获取栈顶
    def getTop(currentSsp: UInt, currentSctr: UInt, currentTOSR: RASPtr, currentTOSW: RASPtr, allowBypass: Boolean):RASEntry = {
      val ret = Wire(new RASEntry)
      if (allowBypass) {  // 如果允许数据旁路
        when (writeBypassValid) { // 开启数据旁路
          ret := writeBypassEntry // 返回旁路数据
        } .elsewhen (TOSRinRange(currentTOSR, currentTOSW)) { // 如果TOSR在范围内
          ret := spec_queue(currentTOSR.value)  // 返回预测栈顶数据
        } .otherwise {  // 如果TOSR不在范围内
          ret := getCommitTop(currentSsp) // 返回提交栈顶数据
        }
      } else {
        when (TOSRinRange(currentTOSR, currentTOSW)) {  // 如果TOSR在范围内
          ret := spec_queue(currentTOSR.value)
        } .otherwise {
          ret := getCommitTop(currentSsp)
        }
      }

      ret
    }


    // it would be unsafe for specPtr manipulation if specSize is not power of 2
    /* 在硬件设计中，特别是涉及到循环缓冲区或队列时，通常要求其大小是 2 的幂。这是因为当大
    小是 2 的幂时，可以通过模运算（%）来实现循环缓冲区的索引，而不用担心整数溢出或缓冲区边
    界错误的问题。模运算可以确保索引在达到缓冲区末尾时循环回到开始位置。

    例如，如果 specSize 是 16（2 的 4 次方），那么当 specPtr 达到 16 时，通过模运算 
    specPtr % 16 会得到 0，这是一个安全的循环。如果 specSize 不是 2 的幂，
    比如 15，模运算可能无法正确地实现循环，因为 15 的任何小于它的 2 的幂（如 8 或 4）
    都不能整除它，这可能导致指针操作中的逻辑错误。

    在 Chisel 代码中，为了确保 specSize 是 2 的幂， 可能会使用 log2Up 函数来计算所需的位
    宽，这个函数会向上取整到最接近的 2 的幂。例如：

    val specSize = 16 // 这是一个 2 的幂
    val specPtrWidth = log2Up(specSize) // specPtrWidth 将会是 4
    在这个例子中，specPtrWidth 计算出来是 4，因为 16 是 2 的 4 次方。这意味着 
    specPtr 需要 4 位来表示，足以循环遍历整个队列的大小。

    总结来说，队列大小必须是 2 的幂的重要性，以确保指针操作的安全性和正确性。
    */
    assert(log2Up(RasSpecSize) == log2Floor(RasSpecSize))

    def ctrMax = ((1l << RasCtrSize) - 1).U // 函数嵌套层数计数器最大值（8层）
    def ptrInc(ptr: UInt) = ptr + 1.U // 指针自增
    def ptrDec(ptr: UInt) = ptr - 1.U // 指针自减

    def specPtrInc(ptr: RASPtr) = ptr + 1.U // 预测栈指针自增
    def specPtrDec(ptr: RASPtr) = ptr - 1.U // 预测栈指针自减


    when (io.redirect_valid && io.redirect_isCall) {  // 如果重定向有效且是CALL
      writeBypassValidWire := true.B  // 写回绕过有效信号为真
      writeBypassValid := true.B  // 写回绕过有效为真
    } .elsewhen (io.redirect_valid) { // 如果仅重定向有效
      // clear current top writeBypass if doing redirect
      writeBypassValidWire := false.B // 写回绕过有效信号为假
      writeBypassValid := false.B   // 写回绕过有效为假
    } .elsewhen (io.s2_fire) {  // s2阶段的预测结果已经准备好
      writeBypassValidWire := io.spec_push_valid  // 写回绕过有效信号为PUSH有效信号
      writeBypassValid := io.spec_push_valid  // 写回绕过有效为PUSH有效
    } .elsewhen (io.s3_fire) {  // s3阶段的预测结果已经准备好
      writeBypassValidWire := false.B
      writeBypassValid := false.B
    } .otherwise {  // 其他情况，旁路有效线性信号等于旁路有效信号
      writeBypassValidWire := writeBypassValid
    }


    val topEntry = getTop(ssp, sctr, TOSR, TOSW, true)  // 获取栈顶
    val topNos = getTopNos(TOSR, true)  // 获取栈顶指针
    val redirectTopEntry = getTop(io.redirect_meta_ssp, io.redirect_meta_sctr, io.redirect_meta_TOSR, io.redirect_meta_TOSW, false) // 获取重定向时的栈顶
    val redirectTopNos = io.redirect_meta_NOS // 获取重定向时的栈顶指针
    val s3TopEntry = getTop(io.s3_meta.ssp, io.s3_meta.sctr, io.s3_meta.TOSR, io.s3_meta.TOSW, false) // 获取S3时的栈顶
    val s3TopNos = io.s3_meta.NOS // 获取S3时的栈顶指针

    val writeEntry = Wire(new RASEntry) // 写入时的栈条目
    val writeNos = Wire(new RASPtr) // 写入时的栈指针
    // 如果重定向有效且是CALL，写入时的栈条目为重定向的地址，否则为输入的预测栈地址
    writeEntry.retAddr := Mux(io.redirect_valid && io.redirect_isCall,  io.redirect_callAddr, io.spec_push_addr)

    // 递归嵌套层数计数器更新，重定向（重定向和重定向call）更新重定向嵌套层数，写入更新写入嵌套层数
    writeEntry.ctr := Mux(io.redirect_valid && io.redirect_isCall,
      Mux(redirectTopEntry.retAddr === io.redirect_callAddr && redirectTopEntry.ctr < ctrMax, io.redirect_meta_sctr + 1.U, 0.U),
      Mux(topEntry.retAddr === io.spec_push_addr && topEntry.ctr < ctrMax, sctr + 1.U, 0.U))

    // 重定向&&Call，写入重定向的栈顶指针，否则写入当前的栈顶指针
    writeNos := Mux(io.redirect_valid && io.redirect_isCall,
      io.redirect_meta_NOS, TOSR)
    // 重定向&&Call 或者 spec_push有效时，写旁路条目和写旁路指针更新
    when (io.spec_push_valid || (io.redirect_valid && io.redirect_isCall)) {
      writeBypassEntry := writeEntry
      writeBypassNos := writeNos
    }

    val realPush = Wire(Bool()) // 真正的Push信号
    val realWriteEntry = Wire(new RASEntry) // 真正的写入条目
    val timingTop = RegInit(0.U.asTypeOf(new RASEntry)) // 正在处理的栈条目？
    val timingNos = RegInit(0.U.asTypeOf(new RASPtr)) // 正在处理的栈指针？

    when (writeBypassValidWire) { // 线性旁路信号有效的时候
      when ((io.redirect_valid && io.redirect_isCall) || io.spec_push_valid) {  // 重定向&&Call 或者 spec_push有效时
        timingTop := writeEntry // 下一个时钟周期的栈条目为写入条目
        timingNos := writeNos // 下一个时钟周期的栈指针为写入指针
      } .otherwise {
        timingTop := writeBypassEntry // 否则，正在处理的栈条目为写旁路条目
        timingNos := writeBypassNos // 正在处理的栈指针为写旁路指针
      }
    } .elsewhen (io.redirect_valid && io.redirect_isRet) {  // 重定向&&Ret
      // getTop using redirect Nos as TOSR
      val popRedSsp = Wire(UInt(log2Up(rasSize).W)) // 存储重定向时的栈指针
      val popRedSctr = Wire(UInt(RasCtrSize.W)) // 存储重定向时的栈计数器
      val popRedTOSR = io.redirect_meta_NOS   // 存储重定向的栈顶写指针
      val popRedTOSW = io.redirect_meta_TOSW  // 存储重定向时的栈顶分配指针

      when (io.redirect_meta_sctr > 0.U) {
      // 重定向有效而且重定向的栈计数器大于 0，说明预测栈有函数递归调用，而且重定向ret有效，则pop的嵌套层数为重定向元数据嵌套层数减一
      // 栈指针使用重定向的pop的栈指针使用重定向元数据的栈指针
        popRedSctr := io.redirect_meta_sctr - 1.U
        popRedSsp := io.redirect_meta_ssp
      } .elsewhen (TOSRinRange(popRedTOSR, TOSW)) {
      // 如果重定向元数据的sctr为0，则没有函数的递归调用,而且重定向的栈顶指针在有效范围内，
      // 则弹出指针为重定向元数据的栈指针减一，弹出计数器为预测栈的栈顶指针的计数器
        popRedSsp := ptrDec(io.redirect_meta_ssp)
        popRedSctr := spec_queue(popRedTOSR.value).ctr  // 这里不应该用commit_stack吗？
      } .otherwise {
      // 如果上面的条件都不满足，说明 popRedTOSR 不在有效范围内，
      // 则栈指针使用当前栈指针减 1，栈计数器使用当前栈指针指向的条目的计数器。
        popRedSsp := ptrDec(io.redirect_meta_ssp)
        popRedSctr := getCommitTop(ptrDec(io.redirect_meta_ssp)).ctr
      }
      // We are deciding top for the next cycle, no need to use bypass here
      // 使用更新后的栈指针和栈计数器，以及重定向信息获取栈顶条目。
      timingTop := getTop(popRedSsp, popRedSctr, popRedTOSR, popRedTOSW, false)
    } .elsewhen (io.redirect_valid) { //只是重定向生效
      // Neither call nor ret
      /* 当前的重定向既不是函数调用（call）也不是函数返回（ret）。
      在处理器的分支预测逻辑中，这可能意味着发生了一个需要特殊处理的事件，
      比如一个跳转（jump）或者由于预测错误导致的修正。
      在这种情况下，timingTop 将被更新为 getTop 返回的值，
      这代表了在当前重定向操作下 RAS 的栈顶条目。这个更新的栈
      顶条目将用于后续的流水线阶段，可能包括错误恢复或者跳转目标的解析。
      */
      val popSsp = io.redirect_meta_ssp
      val popSctr = io.redirect_meta_sctr
      val popTOSR = io.redirect_meta_TOSR
      val popTOSW = io.redirect_meta_TOSW
      // 使用重定向信息获取栈顶条目。
      timingTop := getTop(popSsp, popSctr, popTOSR, popTOSW, false)

    } .elsewhen (io.spec_pop_valid) {
      // getTop using current Nos as TOSR
      // 使用当前的栈顶指针和栈计数器获取栈顶条目。可能是用于回滚？
      val popSsp = Wire(UInt(log2Up(rasSize).W))
      val popSctr = Wire(UInt(RasCtrSize.W))
      val popTOSR = topNos
      val popTOSW = TOSW

      when (sctr > 0.U) {
        // 如果当前的栈计数器大于 0，说明预测栈有函数调用
        // 则使用当前的栈指针，栈计数器减一。
        popSctr := sctr - 1.U
        popSsp := ssp
      } .elsewhen (TOSRinRange(popTOSR, TOSW)) {  // 之前预测的栈顶指针还没弹出去
        popSsp := ptrDec(ssp)
        popSctr := spec_queue(popTOSR.value).ctr
      } .otherwise {  // 之前预测的栈顶指针已经commit了
        popSsp := ptrDec(ssp) //pop的指针为当前指针减一
        popSctr := getCommitTop(ptrDec(ssp)).ctr
      }
      // 确定下一个时钟周期的栈顶，不需要旁路
      // We are deciding top for the next cycle, no need to use bypass here
      timingTop := getTop(popSsp, popSctr, popTOSR, popTOSW, false)
    } .elsewhen (realPush) {
      // just updating spec queue, cannot read from there
      timingTop := realWriteEntry
    } .elsewhen (io.s3_cancel) {
      // s3 is different with s2
      // S3 和 S2 不同，根据s3的数据更新栈顶
      timingTop := getTop(io.s3_meta.ssp, io.s3_meta.sctr, io.s3_meta.TOSR, io.s3_meta.TOSW, false)
      when (io.s3_missed_push) {  // S3判断需要再次PUSH
        val writeEntry_s3 = Wire(new RASEntry)
        timingTop := writeEntry_s3
        writeEntry_s3.retAddr := io.s3_pushAddr
        // 更新写入项的递归层数
        writeEntry_s3.ctr := Mux(timingTop.retAddr === io.s3_pushAddr && io.s3_meta.sctr < ctrMax, io.s3_meta.sctr + 1.U, 0.U)
      } .elsewhen (io.s3_missed_pop) {  // S3判断需要再次POP
        val popRedSsp_s3 = Wire(UInt(log2Up(rasSize).W))
        val popRedSctr_s3 = Wire(UInt(RasCtrSize.W))
        val popRedTOSR_s3 = io.s3_meta.NOS
        val popRedTOSW_s3 = io.s3_meta.TOSW

        when (io.s3_meta.sctr > 0.U) {  // 如果S3的递归层数大于0
          popRedSctr_s3 := io.s3_meta.sctr - 1.U
          popRedSsp_s3 := io.s3_meta.ssp
        } .elsewhen (TOSRinRange(popRedTOSR_s3, popRedTOSW_s3)) {
          popRedSsp_s3 := ptrDec(io.s3_meta.ssp)
          popRedSctr_s3 := spec_queue(popRedTOSR_s3.value).ctr
        } .otherwise {
          popRedSsp_s3 := ptrDec(io.s3_meta.ssp)
          popRedSctr_s3 := getCommitTop(ptrDec(io.s3_meta.ssp)).ctr
        }
        // We are deciding top for the next cycle, no need to use bypass here
        // 使用更新后的栈指针和栈计数器，以及重定向信息获取栈顶条目,不使用旁路
        timingTop := getTop(popRedSsp_s3, popRedSctr_s3, popRedTOSR_s3, popRedTOSW_s3, false)
      }
    } .otherwise {
      // easy case 不需要再次push
      val popSsp = ssp
      val popSctr = sctr
      val popTOSR = TOSR
      val popTOSW = TOSW
      timingTop := getTop(popSsp, popSctr, popTOSR, popTOSW, false)
    }
    val diffTop = Mux(writeBypassValid, writeBypassEntry.retAddr, topEntry.retAddr)

    XSPerfAccumulate("ras_top_mismatch", diffTop =/= timingTop.retAddr);
    /*当RAS（返回地址栈）的顶部条目与预期的顶部条目不一致时，
    这可能是由于栈操作的不匹配或使用仍在处理中的信息更新了提交栈。
    通过监控这种情况，开发者可以诊断和优化硬件的性能问题。*/
    // could diff when more pop than push and a commit stack is updated with inflight info

    // 下一个写入条目的寄存器，通过RegEnable保持，当S2阶段发生或重定向是调用时使能。
    val realWriteEntry_next = RegEnable(writeEntry, io.s2_fire || io.redirect_isCall)
    
    // s3 需要再次push的时候，需要更新的条目和指针
    val s3_missPushEntry = Wire(new RASEntry)
    val s3_missPushAddr = Wire(new RASPtr)
    val s3_missPushNos = Wire(new RASPtr)

    s3_missPushEntry.retAddr := io.s3_pushAddr
    s3_missPushEntry.ctr := Mux(s3TopEntry.retAddr === io.s3_pushAddr && s3TopEntry.ctr < ctrMax, io.s3_meta.sctr + 1.U, 0.U) // 为什么ctr=max的时候也是0？溢出？
    s3_missPushAddr := io.s3_meta.TOSW
    s3_missPushNos := io.s3_meta.TOSR


    // 非重定向Call有效&&s3阶段需要再次push，则下一个写入条目为s3_missPushEntry，否则为realWriteEntry_next
    realWriteEntry := Mux(io.redirect_isCall, realWriteEntry_next,
      Mux(io.s3_missed_push, s3_missPushEntry,
      realWriteEntry_next))
    // 下一个周期的内存分配指针，通过RegEnable保持，当S2阶段发生或重定向是调用时使能。
    val realWriteAddr_next = RegEnable(Mux(io.redirect_valid && io.redirect_isCall, io.redirect_meta_TOSW, TOSW), io.s2_fire || (io.redirect_valid && io.redirect_isCall))
    // 下一个周期的写入指针，通过RegEnable保持，当S2阶段发生或重定向是调用时使能 :这里延迟两个时钟周期是什么意思？
    val realWriteAddr = Mux(io.redirect_isCall, realWriteAddr_next,
      Mux(io.s3_missed_push, s3_missPushAddr,
      realWriteAddr_next))
    // 下一个周期的栈指针，通过RegEnable保持，当S2阶段发生或重定向是调用时使能。
    val realNos_next = RegEnable(Mux(io.redirect_valid && io.redirect_isCall, io.redirect_meta_TOSR, TOSR), io.s2_fire || (io.redirect_valid && io.redirect_isCall))
    val realNos = Mux(io.redirect_isCall, realNos_next,
      Mux(io.s3_missed_push, s3_missPushNos,
      realNos_next))
    // 确实要发生push，当s3阶段需要再次push或者s3阶段需要再次pop且s3阶段需要再次push时，或者重定向有效且重定向是call时
    realPush := (io.s3_fire && (!io.s3_cancel && RegEnable(io.spec_push_valid, io.s2_fire) || io.s3_missed_push)) || RegNext(io.redirect_valid && io.redirect_isCall)
    // 真要push的时候，写入条目和指针
    when (realPush) {
      spec_queue(realWriteAddr.value) := realWriteEntry
      spec_nos(realWriteAddr.value) := realNos
    }
    // 预测栈push
    def specPush(retAddr: UInt, currentSsp: UInt, currentSctr: UInt, currentTOSR: RASPtr, currentTOSW: RASPtr, topEntry: RASEntry) = {
      TOSR := currentTOSW // 栈顶指针 = 内存分配指针
      TOSW := specPtrInc(currentTOSW) // 分配内存指针自增
      // spec sp and ctr should always be maintained
      when (topEntry.retAddr === retAddr && currentSctr < ctrMax) { // 压入的地址一样，即递归调用
        sctr := currentSctr + 1.U // 递归层数自增
      } .otherwise {  // 直接压入新的项，递归嵌套层数为0
        ssp := ptrInc(currentSsp)
        sctr := 0.U
      }
      // if we are draining the capacity of spec queue, force move BOS forward
      // 如果预测栈的容量用完了，强制移动BOS向前
      when (specPtrInc(currentTOSW) === BOS) {
        BOS := specPtrInc(BOS)
        spec_overflowed := true.B;
      }
    }
    // push的时候，写入条目和指针
    when (io.spec_push_valid) {
      specPush(io.spec_push_addr, ssp, sctr, TOSR, TOSW, topEntry)
    }
    // 预测栈pop
    def specPop(currentSsp: UInt, currentSctr: UInt, currentTOSR: RASPtr, currentTOSW: RASPtr, currentTopNos: RASPtr) = {
      // TOSR is only maintained when spec queue is not empty
      // 这注释什么意思？没看懂，为什么说预测栈是空的的时候不需要维护TOSR，如果按照姚老师说的TOSR是栈顶指针的话本来就没必要维护，没必要多此一举打个注释吧
      when (TOSRinRange(currentTOSR, currentTOSW)) {
        TOSR := currentTopNos // currentTopNos 应该是传进来的栈顶指针
      }
      // spec sp and ctr should always be maintained
      when (currentSctr > 0.U) {  // 存在递归，嵌套层数-1
        sctr := currentSctr - 1.U
      } .elsewhen (TOSRinRange(currentTopNos, currentTOSW)) {
        // in range, use inflight data
        ssp := ptrDec(currentSsp)
        sctr := spec_queue(currentTopNos.value).ctr // sctr = 弹出之后栈顶的递归层数
      } .otherwise {
        // NOS not in range, use commit data
        // 提交栈的指针的计数方法和预测栈应该是不一样的
        ssp := ptrDec(currentSsp)
        sctr := getCommitTop(ptrDec(currentSsp)).ctr
        // in overflow state, we cannot determine the next sctr, sctr here is not accurate
        // 注释上表示当栈溢出的时候 sctr无法计算嵌套层数，但是sctr的max值是8，而spec栈有32项，commit栈有16项，即使没有溢出，sctr依然无法计算嵌套层数
        // 这里推测sctr的设计有一些问题，还是说在实际的分支预测的环境下，可以保证递归的层数不超过八层？
        // 但是这样的话为什么要把spec栈和commit栈的大小设计的这么大呢？
      }
    }
    when (io.spec_pop_valid) {  // 预测栈pop
      specPop(ssp, sctr, TOSR, TOSW, topNos)
    }

    // io.spec_pop_addr := Mux(writeBypassValid, writeBypassEntry.retAddr, topEntry.retAddr)
    
    io.spec_pop_addr := timingTop.retAddr // timing 到底是正在处理还是下一个周期？ 看代码实现应该是正在处理，但是他英文注释给的下一个周期
    io.BOS := BOS
    io.TOSW := TOSW
    io.TOSR := TOSR
    io.NOS := topNos  // 栈顶指针，可能是预测栈也可能是提交栈
    io.ssp := ssp
    io.sctr := sctr
    io.nsp := nsp // nsp（Next Stack Pointer）是指向提交栈中下一个要处理的条目的指针，会被ssp覆盖

    when (io.s3_cancel) { // s3和s2结果不一样，要根据s3的结果更新栈顶
      // recovery of all related pointers
      TOSR := io.s3_meta.TOSR
      TOSW := io.s3_meta.TOSW
      ssp := io.s3_meta.ssp
      sctr := io.s3_meta.sctr

      // for missing pop, we also need to do a pop here
      // s3 判断需要再次pop
      when (io.s3_missed_pop) {
        specPop(io.s3_meta.ssp, io.s3_meta.sctr, io.s3_meta.TOSR, io.s3_meta.TOSW, io.s3_meta.NOS)
      }
      when (io.s3_missed_push) {
        // do not use any bypass from f2
        // s3 判断需要再次push
        specPush(io.s3_pushAddr, io.s3_meta.ssp, io.s3_meta.sctr, io.s3_meta.TOSR, io.s3_meta.TOSW, s3TopEntry)
      }
    }

    val commitTop = commit_stack(nsp)
    // 后端发来了ret请求
    when (io.commit_pop_valid) {
      val nsp_update = Wire(UInt(log2Up(rasSize).W))
      when (io.commit_meta_ssp =/= nsp) {
        // force set nsp to commit ssp to avoid permanent errors
        // 强制更新下一个栈指针（nsp）为提交栈指针（commit_ssp）的值。这样做的目的是为了避免由于栈指针不一致而导致的永久性错误。
        nsp_update := io.commit_meta_ssp
      } .otherwise {
        nsp_update := nsp
      }

      // if ctr > 0, --ctr in stack, otherwise --nsp
      when (commitTop.ctr > 0.U) {
        commit_stack(nsp_update).ctr := commitTop.ctr - 1.U
        nsp := nsp_update
      } .otherwise {
        nsp := ptrDec(nsp_update);
      }
      // XSError(io.commit_meta_ssp =/= nsp, "nsp mismatch with expected ssp")
    }

    val commit_push_addr = spec_queue(io.commit_meta_TOSW.value).retAddr



    when (io.commit_push_valid) { // 后端发来了call请求
      val nsp_update = Wire(UInt(log2Up(rasSize).W))
      when (io.commit_meta_ssp =/= nsp) {
        // force set nsp to commit ssp to avoid permanent errors
        nsp_update := io.commit_meta_ssp
      } .otherwise {
        nsp_update := nsp
      }
      // if ctr < max && topAddr == push addr, ++ctr, otherwise ++nsp
      // 按照注释，topAddr == push addr的时候是不用push的，但是验证的时候spec栈是进行了push的，是因为spec栈和commit栈不一样？
      when (commitTop.ctr < ctrMax && commitTop.retAddr === commit_push_addr) {
        commit_stack(nsp_update).ctr := commitTop.ctr + 1.U
        nsp := nsp_update
      } .otherwise {
        nsp := ptrInc(nsp_update)
        commit_stack(ptrInc(nsp_update)).retAddr := commit_push_addr
        commit_stack(ptrInc(nsp_update)).ctr := 0.U
      }
      // when overflow, BOS may be forced move forward, do not revert those changes
      // spec栈溢出的时候，BOS栈底指针会递增，即用一个循环栈来模拟spec栈
      when (!spec_overflowed || isAfter(specPtrInc(io.commit_meta_TOSW), BOS)) {
        BOS := specPtrInc(io.commit_meta_TOSW)
        spec_overflowed := false.B
      }

      // XSError(io.commit_meta_ssp =/= nsp, "nsp mismatch with expected ssp")
      // XSError(io.commit_push_addr =/= commit_push_addr, "addr from commit mismatch with addr from spec")
    }

    when (io.redirect_valid) {  // 重定向的时候一定没有commit，所以只用操作spec栈
      TOSR := io.redirect_meta_TOSR
      TOSW := io.redirect_meta_TOSW
      ssp := io.redirect_meta_ssp
      sctr := io.redirect_meta_sctr

      when (io.redirect_isCall) {
        specPush(io.redirect_callAddr, io.redirect_meta_ssp, io.redirect_meta_sctr, io.redirect_meta_TOSR, io.redirect_meta_TOSW, redirectTopEntry)
      }
      when (io.redirect_isRet) {
        specPop(io.redirect_meta_ssp, io.redirect_meta_sctr, io.redirect_meta_TOSR, io.redirect_meta_TOSW, redirectTopNos)
      }
    }

    io.debug.commit_stack.zipWithIndex.foreach{case (a, i) => a := commit_stack(i)}
    io.debug.spec_nos.zipWithIndex.foreach{case (a, i) => a := spec_nos(i)}
    io.debug.spec_queue.zipWithIndex.foreach{ case (a, i) => a := spec_queue(i)}
  }
  // RAS 操作

  val stack = Module(new RASStack(RasSize, RasSpecSize)).io

  val s2_spec_push = WireInit(false.B)
  val s2_spec_pop = WireInit(false.B)
  val s2_full_pred = io.in.bits.resp_in(0).s2.full_pred(2)
  // when last inst is an rvi call, fall through address would be set to the middle of it, so an addition is needed
  // s2_full_pred.last_may_be_rvi_call 是否为压缩指令
  val s2_spec_new_addr = s2_full_pred.fallThroughAddr + Mux(s2_full_pred.last_may_be_rvi_call, 2.U, 0.U)

  stack.spec_push_valid := s2_spec_push
  stack.spec_pop_valid  := s2_spec_pop
  stack.spec_push_addr := s2_spec_new_addr

  // confirm that the call/ret is the taken cfi
  s2_spec_push := io.s2_fire(2) && s2_full_pred.hit_taken_on_call && !io.s3_redirect(2)
  s2_spec_pop  := io.s2_fire(2) && s2_full_pred.hit_taken_on_ret  && !io.s3_redirect(2)

  //val s2_jalr_target = io.out.s2.full_pred.jalr_target
  //val s2_last_target_in = s2_full_pred.targets.last
  // val s2_last_target_out = io.out.s2.full_pred(2).targets.last
  val s2_is_jalr = s2_full_pred.is_jalr
  val s2_is_ret = s2_full_pred.is_ret
  val s2_top = stack.spec_pop_addr
  // assert(is_jalr && is_ret || !is_ret)
  when(s2_is_ret && io.ctrl.ras_enable) {
    io.out.s2.full_pred.map(_.jalr_target).foreach(_ := s2_top)
    // FIXME: should use s1 globally
  }
  //s2_last_target_out := Mux(s2_is_jalr, s2_jalr_target, s2_last_target_in)
  io.out.s2.full_pred.zipWithIndex.foreach{ case (a, i) =>
    a.targets.last := Mux(s2_is_jalr, io.out.s2.full_pred(i).jalr_target, io.in.bits.resp_in(0).s2.full_pred(i).targets.last)
  }

  val s2_meta = Wire(new RASMeta)
  s2_meta.ssp := stack.ssp
  s2_meta.sctr := stack.sctr
  s2_meta.TOSR := stack.TOSR
  s2_meta.TOSW := stack.TOSW
  s2_meta.NOS := stack.NOS

  val s3_top = RegEnable(stack.spec_pop_addr, io.s2_fire(2))
  val s3_spec_new_addr = RegEnable(s2_spec_new_addr, io.s2_fire(2))

  // val s3_jalr_target = io.out.s3.full_pred.jalr_target
  // val s3_last_target_in = io.in.bits.resp_in(0).s3.full_pred(2).targets.last
  // val s3_last_target_out = io.out.s3.full_pred(2).targets.last
  val s3_is_jalr = io.in.bits.resp_in(0).s3.full_pred(2).is_jalr
  val s3_is_ret = io.in.bits.resp_in(0).s3.full_pred(2).is_ret
  // assert(is_jalr && is_ret || !is_ret)
  when(s3_is_ret && io.ctrl.ras_enable) {
    io.out.s3.full_pred.map(_.jalr_target).foreach(_ := s3_top)
    // FIXME: should use s1 globally
  }
  // s3_last_target_out := Mux(s3_is_jalr, s3_jalr_target, s3_last_target_in)
  io.out.s3.full_pred.zipWithIndex.foreach{ case (a, i) =>
    a.targets.last := Mux(s3_is_jalr, io.out.s3.full_pred(i).jalr_target, io.in.bits.resp_in(0).s3.full_pred(i).targets.last)
  }

  val s3_pushed_in_s2 = RegEnable(s2_spec_push, io.s2_fire(2))
  val s3_popped_in_s2 = RegEnable(s2_spec_pop,  io.s2_fire(2))
  val s3_push = io.in.bits.resp_in(0).s3.full_pred(2).hit_taken_on_call
  val s3_pop  = io.in.bits.resp_in(0).s3.full_pred(2).hit_taken_on_ret

  val s3_cancel = io.s3_fire(2) && (s3_pushed_in_s2 =/= s3_push || s3_popped_in_s2 =/= s3_pop)
  stack.s2_fire := io.s2_fire(2)
  stack.s3_fire := io.s3_fire(2)

  stack.s3_cancel := s3_cancel

  val s3_meta = RegEnable(s2_meta, io.s2_fire(2))

  stack.s3_meta := s3_meta
  stack.s3_missed_pop := s3_pop && !s3_popped_in_s2
  stack.s3_missed_push := s3_push && !s3_pushed_in_s2
  stack.s3_pushAddr := s3_spec_new_addr

  // no longer need the top Entry, but TOSR, TOSW, ssp sctr
  // TODO: remove related signals
  io.out.last_stage_spec_info.sctr  := s3_meta.sctr
  io.out.last_stage_spec_info.ssp := s3_meta.ssp
  io.out.last_stage_spec_info.TOSW := s3_meta.TOSW
  io.out.last_stage_spec_info.TOSR := s3_meta.TOSR
  io.out.last_stage_spec_info.NOS := s3_meta.NOS
  io.out.last_stage_spec_info.topAddr := s3_top
  io.out.last_stage_meta := s3_meta.asUInt


  val redirect = RegNextWithEnable(io.redirect)
  val do_recover = redirect.valid
  val recover_cfi = redirect.bits.cfiUpdate

  val retMissPred  = do_recover && redirect.bits.level === 0.U && recover_cfi.pd.isRet
  val callMissPred = do_recover && redirect.bits.level === 0.U && recover_cfi.pd.isCall
  // when we mispredict a call, we must redo a push operation
  // similarly, when we mispredict a return, we should redo a pop
  stack.redirect_valid := do_recover
  stack.redirect_isCall := callMissPred
  stack.redirect_isRet := retMissPred
  stack.redirect_meta_ssp := recover_cfi.ssp
  stack.redirect_meta_sctr := recover_cfi.sctr
  stack.redirect_meta_TOSW := recover_cfi.TOSW
  stack.redirect_meta_TOSR := recover_cfi.TOSR
  stack.redirect_meta_NOS := recover_cfi.NOS
  stack.redirect_callAddr := recover_cfi.pc + Mux(recover_cfi.pd.isRVC, 2.U, 4.U)

  val update = io.update.bits
  val updateMeta = io.update.bits.meta.asTypeOf(new RASMeta)
  val updateValid = io.update.valid

  stack.commit_push_valid := updateValid && update.is_call_taken
  stack.commit_pop_valid := updateValid && update.is_ret_taken
  stack.commit_push_addr := update.ftb_entry.getFallThrough(update.pc) + Mux(update.ftb_entry.last_may_be_rvi_call, 2.U, 0.U)
  stack.commit_meta_TOSW := updateMeta.TOSW
  stack.commit_meta_TOSR := updateMeta.TOSR
  stack.commit_meta_ssp := updateMeta.ssp
  stack.commit_meta_sctr := updateMeta.sctr


  XSPerfAccumulate("ras_s3_cancel", s3_cancel)
  XSPerfAccumulate("ras_redirect_recover", redirect.valid)
  XSPerfAccumulate("ras_s3_and_redirect_recover_at_the_same_time", s3_cancel && redirect.valid)


  val spec_debug = stack.debug
  XSDebug(io.s2_fire(2), "----------------RAS----------------\n")
  XSDebug(io.s2_fire(2), " TopRegister: 0x%x\n",stack.spec_pop_addr)
  XSDebug(io.s2_fire(2), "  index       addr           ctr           nos (spec part)\n")
  for(i <- 0 until RasSpecSize){
      XSDebug(io.s2_fire(2), "  (%d)   0x%x      %d       %d",i.U,spec_debug.spec_queue(i).retAddr,spec_debug.spec_queue(i).ctr, spec_debug.spec_nos(i).value)
      when(i.U === stack.TOSW.value){XSDebug(io.s2_fire(2), "   <----TOSW")}
      when(i.U === stack.TOSR.value){XSDebug(io.s2_fire(2), "   <----TOSR")}
      when(i.U === stack.BOS.value){XSDebug(io.s2_fire(2), "   <----BOS")}
      XSDebug(io.s2_fire(2), "\n")
  }
  XSDebug(io.s2_fire(2), "  index       addr           ctr   (committed part)\n")
  for(i <- 0 until RasSize){
      XSDebug(io.s2_fire(2), "  (%d)   0x%x      %d",i.U,spec_debug.commit_stack(i).retAddr,spec_debug.commit_stack(i).ctr)
      when(i.U === stack.ssp){XSDebug(io.s2_fire(2), "   <----ssp")}
      when(i.U === stack.nsp){XSDebug(io.s2_fire(2), "   <----nsp")}
      XSDebug(io.s2_fire(2), "\n")
  }
  /*
  XSDebug(s2_spec_push, "s2_spec_push  inAddr: 0x%x  inCtr: %d |  allocNewEntry:%d |   sp:%d \n",
  s2_spec_new_addr,spec_debug.spec_push_entry.ctr,spec_debug.spec_alloc_new,spec_debug.sp.asUInt)
  XSDebug(s2_spec_pop, "s2_spec_pop  outAddr: 0x%x \n",io.out.s2.getTarget)
  val s3_recover_entry = spec_debug.recover_push_entry
  XSDebug(s3_recover && s3_push, "s3_recover_push  inAddr: 0x%x  inCtr: %d |  allocNewEntry:%d |   sp:%d \n",
    s3_recover_entry.retAddr, s3_recover_entry.ctr, spec_debug.recover_alloc_new, s3_sp.asUInt)
  XSDebug(s3_recover && s3_pop, "s3_recover_pop  outAddr: 0x%x \n",io.out.s3.getTarget)
  val redirectUpdate = redirect.bits.cfiUpdate
  XSDebug(do_recover && callMissPred, "redirect_recover_push\n")
  XSDebug(do_recover && retMissPred, "redirect_recover_pop\n")
  XSDebug(do_recover, "redirect_recover(SP:%d retAddr:%x ctr:%d) \n",
      redirectUpdate.rasSp,redirectUpdate.rasEntry.retAddr,redirectUpdate.rasEntry.ctr)
  */

  generatePerfEvent()
}
