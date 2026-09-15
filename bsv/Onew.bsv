package Onew;

// 1-Wire 主机：寄存器组、微秒节拍、一个按时隙推进的引擎。时隙多长、第几微秒采样在
// OnewSlot（BH）里照 AN126 一张表写死；这里只管一个时隙接一个时隙地走，以及字节操作里
// 下一个时隙挑哪一种。收到的字节用 hwcore 的 Gf2 顺手算 CRC-8/MAXIM-DOW。

import RegIf::*;
import OnewRegs::*;
import OnewSlot::*;
import Gf2::*;

typedef struct {
  Bool crc;
} OnewCfg;

interface OnewPins;
  (* always_ready, result = "ow_pull" *) method Bit#(1) ow_pull;
  (* always_ready, always_enabled, prefix = "" *)
  method Action ow_in((* port = "ow_i" *) Bit#(1) v);
endinterface

interface OnewIfc#(numeric type aw, numeric type dw);
  interface RegIf#(aw, dw) regs;
  interface OnewPins        pins;
  (* always_ready *) method Bool irq;
endinterface

module mkOnew#(OnewCfg cfg)(OnewIfc#(aw, dw))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 1, dw),
              Add#(_c, 3, dw), Add#(_d, 8, dw), Add#(_e, 16, dw));

  OnewRegsIfc#(aw, dw) r <- mkOnewRegs(OnewRegsCfg { crc: cfg.crc });

  Wire#(Bit#(1)) owIn <- mkBypassWire;
  // 写脉冲在总线方法之后才有，引擎却要在它之前读 txd：隔一拍，由 CReg 递过去
  Reg#(Maybe#(Bit#(3))) pend[2] <- mkCReg(2, tagged Invalid);

  Reg#(Bit#(16))  sub  <- mkReg(0);
  Reg#(Bool)      busy <- mkReg(False);
  Reg#(Slot)      slot <- mkReg(Reset);
  Reg#(UInt#(10)) t    <- mkReg(0);
  Reg#(UInt#(4))  left <- mkReg(0);
  Reg#(Bit#(3))   opR  <- mkReg(0);
  Reg#(Bit#(8))   sh   <- mkReg(0);
  Reg#(Bit#(1))   pres <- mkReg(0);
  Reg#(Bit#(8))   rx   <- mkReg(0);

  // 特性关掉就不例化：恒零的只读寄存器占位，综合器整片消掉
  function Reg#(Bit#(8)) zero8 = interface Reg;
    method Bit#(8) _read = 0;
    method Action _write(Bit#(8) x) = noAction;
  endinterface;
  Reg#(Bit#(8)) crcR = zero8;
  if (cfg.crc) crcR <- mkReg(0);

  rule mark;
    if (r.cmd_op_wr) pend[1] <= tagged Valid r.cmd_op_wr_val;
  endrule

  // 节拍、开始与推进写的是同一批寄存器，所以合成一条规则。时隙开头把节拍清零，
  // 拉低的时长于是正好是 low × (tick + 1) 拍
  rule engine;
    Timing tm = timing(slot);
    pend[0] <= tagged Invalid;   // 忙着的时候写进来的命令不算
    if (!busy) begin
      sub <= 0;
      if (pend[0] matches tagged Valid .op &&& op <= 4) begin
        Bit#(8) d = r.txd;
        busy <= True;
        t    <= 0;
        opR  <= op;
        sh   <= d;
        case (op)
          0: begin slot <= Reset;           left <= 1; end
          1: begin slot <= writeSlot(d[0]); left <= 1; end
          2: begin slot <= ReadBit;         left <= 1; end
          3: begin slot <= writeSlot(d[0]); left <= 8; end
          default: begin slot <= ReadBit;   left <= 8; end
        endcase
      end
    end else begin
      Bool us = sub == r.tick;
      sub <= us ? 0 : sub + 1;
      if (us) begin
        Bit#(8) s = sh;
        if (tm.sample != 0 && t == tm.sample) begin
          if (slot == Reset) pres <= ~owIn;   // 线被拉低就是有从机应答
          else s = {owIn, sh[7:1]};           // 读：低位先到
        end
        if (t + 1 == tm.total) begin
          if (left == 1) begin
            busy <= False;
            r.status_done_set(1);
            case (opR)
              0: crcR <= 0;                   // 一次复位开始一段新的校验
              2: rx <= zeroExtend(s[7]);
              4: begin rx <= s; crcR <= crcByte(crc8MaximDow, crcR, s); end
            endcase
            sh <= s;
          end else begin
            Bit#(8) ns = (opR == 3) ? s >> 1 : s;
            if (opR == 3) slot <= writeSlot(ns[0]);
            t    <= 0;
            left <= left - 1;
            sh   <= ns;
          end
        end else begin
          t  <= t + 1;
          sh <= s;
        end
      end
    end
  endrule

  rule show;
    r.status_busy_in(busy ? 1 : 0);
    r.status_presence_in(pres);
    r.rxd_in(rx);
    r.crc_in(crcFinal(crc8MaximDow, crcR));
  endrule

  interface regs = r.regs;
  interface OnewPins pins;
    method Bit#(1) ow_pull = (busy && t < timing(slot).low) ? 1 : 0;
    method Action ow_in(Bit#(1) v); owIn._write(v); endmethod
  endinterface
  method Bool irq = r.status_done == 1 && r.ctrl_ien == 1;
endmodule

endpackage
