"""onew 的行为测试台：主机对着一个 1-Wire 从机模型复位、写命令、读 ROM 号。

时序照 Maxim AN126 表 1 的标准速度。`tick` 写 0，一拍就是一微秒，拉低多久可以逐拍量；
最后把 `tick` 改成 2 再复位一次，拉低必须变成三倍。

从机模型只看主机拉没拉线（不看总线），于是自己的应答不会被当成新的时隙：
  · 主机拉低超过 300 微秒再放开是复位：放开后第 30 微秒起拉低 120 微秒作应答
  · 应答之后收命令：主机每个下降沿后第 30 微秒采样，主机还拉着就是 0
  · 收到 0x33（Read ROM）就交出 8 字节 ROM 号，低位先出：要交 0 的时隙从下降沿起
    拉住 30 微秒，盖过主机第 15 微秒的采样

ROM 号的第 8 字节是前 7 字节的 CRC-8/MAXIM-DOW。算它的参考实现生成之前先对着 RevEng
目录的校验值 0xa1 自证一遍，不对就不生成。

认矩阵：`crc` 关着时不查 CRC 寄存器。命令序列用 StmtFSM 写：这是一串先后分明的步骤，
正是它的长处；照 DatenLord 笔记 04-02 用 mkFSM 手动 start，不用会提前 $finish 的 mkAutoFSM。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
knobs = cfg.get("knobs", {})
crc_on = bool(knobs.get("crc", True))


def crc8(data):
    c = 0
    for byte in data:
        for i in range(8):
            bit = (byte >> i) & 1
            top = (c >> 7) & 1
            c = ((c << 1) & 0xFF) ^ (0x31 if top ^ bit else 0)
    return int(f"{c:08b}"[::-1], 2)


if crc8(b"123456789") != 0xA1:
    raise SystemExit("CRC-8/MAXIM-DOW 参考实现对不上目录的 0xa1，不生成测试台")

ROM = [0x28, 0xA2, 0x3B, 0x5C, 0x01, 0x00, 0x00]
ROM.append(crc8(ROM))
rom_bits = sum(b << (8 * i) for i, b in enumerate(ROM))
rom_cases = "\n".join(f"      {i}: return 8'h{b:02X};" for i, b in enumerate(ROM))

CTRL, TICK, TXD, CMD, STATUS, RXD, CRC = 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18

crc_mid = f"""
        if (i == 6) seq
          rd(8'h{CRC:02X});
          action
            if (rdR[7:0] != 8'h{ROM[7]:02X}) begin
              $display("FAIL after seven ROM bytes the crc register reads %02h, want %02h", rdR[7:0], 8'h{ROM[7]:02X});
              bad <= True;
            end
          endaction
        endseq""" if crc_on else ""
crc_end = f"""
    rd(8'h{CRC:02X});
    action
      if (rdR[7:0] != 0) begin
        $display("FAIL after all eight ROM bytes the crc register reads %02h, want 00", rdR[7:0]);
        bad <= True;
      end
    endaction""" if crc_on else ""

verdict = ("reset gets a presence, the timing follows AN126 at one and three cycles per microsecond, "
           "Read ROM returns the device ROM number, done and irq behave, and no device means no presence"
           + (", and the crc register checks the ROM number" if crc_on else ""))

TEMPLATE = r'''package Onew@L@Tb;

// 由 htest/mkonewtb.py 生成，勿手改。这一点：crc=@CRCON@

import StmtFSM::*;
import ConfigReg::*;
import RegIf::*;
import Onew::*;

typedef enum { Idle, Cmd, Rom } Model deriving (Bits, Eq);

Bit#(64) romBits = 64'h@ROMBITS@;

function Bit#(8) romByte(UInt#(4) i);
  case (i)
@ROMCASES@
    default: return 0;
  endcase
endfunction

(* synthesize *)
module mkOnew@L@Tb(Empty);
  OnewIfc#(8, 32) d <- mkOnew(OnewCfg { crc: @CRC@ });

  // ---- 从机模型 ----
  Reg#(Bool)      present <- mkReg(True);
  Reg#(Bit#(1))   mPrev   <- mkReg(0);
  Reg#(UInt#(16)) mRun    <- mkReg(0);
  Reg#(UInt#(16)) since   <- mkReg(0);
  Reg#(Bool)      armed   <- mkReg(False);
  Reg#(UInt#(16)) presAt  <- mkReg(0);
  Reg#(UInt#(8))  hold    <- mkReg(0);
  Reg#(Model)     ms      <- mkReg(Idle);
  Reg#(UInt#(7))  nbit    <- mkReg(0);
  Reg#(Bit#(8))   cmdSh   <- mkReg(0);
  Reg#(Bit#(8))   cmdGot[2] <- mkCReg(2, 0);
  Reg#(Bit#(1))   sp      <- mkReg(0);

  rule bus;
    d.pins.ow_in((d.pins.ow_pull == 1 || sp == 1) ? 0 : 1);
  endrule

  rule slave;
    Bit#(1) m = d.pins.ow_pull;
    Bool fall = m == 1 && mPrev == 0;
    Bool rise = m == 0 && mPrev == 1;
    mPrev <= m;
    mRun  <= (m == 1) ? mRun + 1 : 0;
    UInt#(16) s = fall ? 0 : since + 1;
    since <= s;
    Bool rst = rise && mRun >= 300;
    UInt#(16) pa = rst ? 1 : ((presAt > 0 && presAt < 150) ? presAt + 1 : 0);
    presAt <= pa;
    UInt#(8) h = (hold > 0) ? hold - 1 : 0;
    Bool arm = armed;
    if (rst) begin
      ms <= Cmd; nbit <= 0; arm = False;
    end else if (fall && ms != Idle) begin
      arm = True;
      if (ms == Rom && romBits[nbit] == 0) h = 30;
    end else if (armed && s == 30) begin
      arm = False;
      if (ms == Cmd) begin
        Bit#(8) nx = {(m == 1) ? 1'b0 : 1'b1, cmdSh[7:1]};
        cmdSh <= nx;
        if (nbit == 7) begin
          cmdGot[0] <= nx;
          ms <= (nx == 8'h33) ? Rom : Cmd;
          nbit <= 0;
        end else nbit <= nbit + 1;
      end else if (ms == Rom) nbit <= nbit + 1;
    end
    armed <= arm;
    hold  <= h;
    sp <= (present && ((pa >= 30 && pa < 150) || h > 0)) ? 1 : 0;
  endrule

  // ---- 量主机每次拉低多久 ----
  Reg#(UInt#(16)) lowRun     <- mkReg(0);
  Reg#(UInt#(16)) lastLow[2] <- mkCReg(2, 0);
  Reg#(UInt#(16)) minLow[2]  <- mkCReg(2, maxBound);
  Reg#(UInt#(16)) maxLow[2]  <- mkCReg(2, 0);

  rule monitor;
    if (d.pins.ow_pull == 1) lowRun <= lowRun + 1;
    else if (lowRun > 0) begin
      lastLow[0] <= lowRun;
      if (lowRun < minLow[0]) minLow[0] <= lowRun;
      if (lowRun > maxLow[0]) maxLow[0] <= lowRun;
      lowRun <= 0;
    end
  endrule

  // ---- 命令序列 ----
  Reg#(Bool)     bad   <- mkReg(False);
  Reg#(Bit#(32)) rdR   <- mkReg(0);
  Reg#(Bool)     busyR <- mkReg(False);
  Reg#(UInt#(4)) i     <- mkReg(0);
  Reg#(UInt#(32)) cyc  <- mkConfigReg(0);
  Reg#(UInt#(32)) markC <- mkReg(0);

  function Action wr(Bit#(8) a, Bit#(32) v) = action
    let _ <- d.regs.access(RegReq { addr: a, write: True, wdata: v, wstrb: 4'hF });
  endaction;

  function Action rd(Bit#(8) a) = action
    let x <- d.regs.access(RegReq { addr: a, write: False, wdata: 0, wstrb: 4'hF });
    rdR <= x.rdata;
  endaction;

  function Stmt op(Bit#(32) code) = seq
    wr(8'h@CMD@, code);
    delay(3);
    busyR <= True;
    while (busyR) action
      let x <- d.regs.access(RegReq { addr: 8'h@STATUS@, write: False, wdata: 0, wstrb: 4'hF });
      busyR <= x.rdata[0] == 1;
    endaction
  endseq;

  Stmt test = seq
    wr(8'h@TICK@, 0);

    // 复位：从机在，应答要读得到；主机拉低正好 480 微秒
    op(0);
    rd(8'h@STATUS@);
    // 并列的 if 各写一次 bad 是并行冲突（G0004），攒进局部变量再写一次
    action
      Bool wrong = False;
      if (rdR[1] != 1) begin $display("FAIL reset with a device present reads no presence"); wrong = True; end
      if (lastLow[1] != 480) begin $display("FAIL the reset pulse is %0d microseconds, want 480", lastLow[1]); wrong = True; end
      if (wrong) bad <= True;
    endaction

    // 写 0x33：从机收到的就是它；写 1 拉低 6 微秒、写 0 拉低 60 微秒
    wr(8'h@TXD@, 32'h33);
    action minLow[1] <= maxBound; maxLow[1] <= 0; endaction
    op(3);
    action
      Bool wrong = False;
      if (cmdGot[1] != 8'h33) begin $display("FAIL the device received command %02h, want 33", cmdGot[1]); wrong = True; end
      if (minLow[1] != 6) begin $display("FAIL a write-1 slot pulls low %0d microseconds, want 6", minLow[1]); wrong = True; end
      if (maxLow[1] != 60) begin $display("FAIL a write-0 slot pulls low %0d microseconds, want 60", maxLow[1]); wrong = True; end
      if (wrong) bad <= True;
    endaction

    // 读 8 字节 ROM 号
    for (i <= 0; i < 8; i <= i + 1) seq
      op(4);
      rd(8'h@RXD@);
      action
        if (rdR[7:0] != romByte(i)) begin
          $display("FAIL ROM byte %0d reads %02h, want %02h", i, rdR[7:0], romByte(i));
          bad <= True;
        end
      endaction@CRCMID@
    endseq@CRCEND@

    // done 置位；没使能不抬中断，使能后抬，写一清掉
    rd(8'h@STATUS@);
    action if (rdR[2] != 1) begin $display("FAIL done is not set after an operation"); bad <= True; end endaction
    action if (d.irq) begin $display("FAIL irq rises with done set but ien off"); bad <= True; end endaction
    wr(8'h@CTRL@, 1);
    action if (!d.irq) begin $display("FAIL irq stays low with done set and ien on"); bad <= True; end endaction
    wr(8'h@STATUS@, 4);
    action if (d.irq) begin $display("FAIL irq stays high after done is cleared"); bad <= True; end endaction
    wr(8'h@CTRL@, 0);

    // 拿掉从机：复位读不到应答
    present <= False;
    op(0);
    rd(8'h@STATUS@);
    action if (rdR[1] != 0) begin $display("FAIL reset with no device reads a presence"); bad <= True; end endaction
    present <= True;

    // 一微秒三拍：复位拉低 1440 拍
    wr(8'h@TICK@, 2);
    op(0);
    action if (lastLow[1] != 1440) begin $display("FAIL at three cycles per microsecond the reset pulse is %0d cycles, want 1440", lastLow[1]); bad <= True; end endaction

    // 操作进行中把 tick 改小：节拍计数已经越过新的 tick 也不许卡住。tick 为 9 时计数在 0 到 9
    // 之间转，两次各晚一拍改成 0，两个相邻的相位里至少有一次计数大于 0。复位还剩约 760 微秒，
    // 改完一拍一微秒，两万拍内必须做完
    wr(8'h@TICK@, 9);
    markC <= cyc;
    wr(8'h@CMD@, 0);
    delay(2000);
    wr(8'h@TICK@, 0);
    busyR <= True;
    while (busyR) action
      let x <- d.regs.access(RegReq { addr: 8'h@STATUS@, write: False, wdata: 0, wstrb: 4'hF });
      busyR <= x.rdata[0] == 1;
    endaction
    action if (cyc - markC > 20000) begin $display("FAIL after tick is lowered mid operation the reset takes %0d cycles", cyc - markC); bad <= True; end endaction
    wr(8'h@TICK@, 9);
    markC <= cyc;
    wr(8'h@CMD@, 0);
    delay(2001);
    wr(8'h@TICK@, 0);
    busyR <= True;
    while (busyR) action
      let x <- d.regs.access(RegReq { addr: 8'h@STATUS@, write: False, wdata: 0, wstrb: 4'hF });
      busyR <= x.rdata[0] == 1;
    endaction
    action if (cyc - markC > 20000) begin $display("FAIL after tick is lowered mid operation the reset takes %0d cycles", cyc - markC); bad <= True; end endaction
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule tick_;
    cyc <= cyc + 1;
    if (cyc > 100000) begin
      $display("TIMEOUT");
      $finish(1);
    end
  endrule

  rule fin (started && fsm.done);
    if (bad) $display("FAILED");
    else $display("PASS onew: @VERDICT@");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

txt = (TEMPLATE.replace("@L@", label)
       .replace("@CRCON@", str(crc_on))
       .replace("@CRC@", "True" if crc_on else "False")
       .replace("@ROMBITS@", f"{rom_bits:016X}")
       .replace("@ROMCASES@", rom_cases)
       .replace("@CRCMID@", crc_mid)
       .replace("@CRCEND@", crc_end)
       .replace("@VERDICT@", verdict)
       .replace("@CTRL@", f"{CTRL:02X}").replace("@TICK@", f"{TICK:02X}")
       .replace("@TXD@", f"{TXD:02X}").replace("@CMD@", f"{CMD:02X}")
       .replace("@STATUS@", f"{STATUS:02X}").replace("@RXD@", f"{RXD:02X}"))

(out / f"Onew{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  onew 行为测试台就位：crc={crc_on}，ROM 号 {' '.join(f'{b:02X}' for b in ROM)}")
