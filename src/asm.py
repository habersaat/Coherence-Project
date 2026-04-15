#!/usr/bin/env python3
"""
Minimal RV32I assembler for MSI cache coherence workloads.
Handles: lw, sw, addi, add, slli, jal, bne
"""

REGS = {
    'zero':0,'ra':1,'sp':2,'gp':3,'tp':4,
    't0':5,'t1':6,'t2':7,'s0':8,'s1':9,
    'a0':10,'a1':11,'a2':12,'a3':13,'a4':14,'a5':15,
    'a6':16,'a7':17,'s2':18,'s3':19,'s4':20,'s5':21,
    's6':22,'s7':23,'s8':24,'s9':25,'s10':26,'s11':27,
    't3':28,'t4':29,'t5':30,'t6':31
}
def r(n): return REGS[n]

def lw_enc(rd, imm, rs1):
    return ((imm&0xFFF)<<20)|(r(rs1)<<15)|(2<<12)|(r(rd)<<7)|0x03

def sw_enc(rs2, imm, rs1):
    m=imm&0xFFF
    return ((m>>5)<<25)|(r(rs2)<<20)|(r(rs1)<<15)|(2<<12)|((m&0x1F)<<7)|0x23

def addi_enc(rd, rs1, imm):
    return ((imm&0xFFF)<<20)|(r(rs1)<<15)|(0<<12)|(r(rd)<<7)|0x13

def add_enc(rd, rs1, rs2):
    return (r(rs2)<<20)|(r(rs1)<<15)|(r(rd)<<7)|0x33

def slli_enc(rd, rs1, shamt):
    return (shamt<<20)|(r(rs1)<<15)|(1<<12)|(r(rd)<<7)|0x13

def jal_enc(rd, offset):
    m=offset&0x1FFFFF
    return ((m>>20)&1)<<31 | ((m>>1)&0x3FF)<<21 | ((m>>11)&1)<<20 | ((m>>12)&0xFF)<<12 | (r(rd)<<7) | 0x6F

def bne_enc(rs1, rs2, offset):
    m=offset&0x1FFF
    return ((m>>12)&1)<<31 | ((m>>5)&0x3F)<<25 | r(rs2)<<20 | r(rs1)<<15 | (1<<12) | ((m>>1)&0xF)<<8 | ((m>>11)&1)<<7 | 0x63

# ---- Verify against known encodings from top_fpga.v ----
assert addi_enc('sp','sp',-16) == 0xff010113, f"{addi_enc('sp','sp',-16):#010x}"
assert lw_enc('a5',0x100,'zero') == 0x10002783
assert addi_enc('a4','a5',1)   == 0x00178713
assert sw_enc('a4',0x100,'zero') == 0x10e02023
assert jal_enc('zero',-12)     == 0xff5ff06f
print("Verification passed: all 5 known encodings correct")

# ---- Shared header: ID assignment + chunk address setup ----
# Used by workloads 4 and 5.
# After this (9 instructions, 0x000-0x020):
#   s0 = my core ID (0-3)
#   s1 = base address of my 4-element chunk (0x300 + ID*16)
def id_and_chunk_header():
    p = []
    p.append((addi_enc('t2','zero',0x200),  "addi t2,zero,0x200  # t2 = ID counter addr"))
    p.append((lw_enc('s0',0,'t2'),          "lw   s0,0(t2)       # [retry] s0=counter"))
    p.append((addi_enc('t3','s0',1),         "addi t3,s0,1        # t3=s0+1"))
    p.append((sw_enc('t3',0,'t2'),           "sw   t3,0(t2)       # write back"))
    p.append((lw_enc('t4',0,'t2'),           "lw   t4,0(t2)       # re-read verify"))
    p.append((bne_enc('t4','t3',-16),        "bne  t4,t3,-16      # retry if changed"))
    p.append((slli_enc('s1','s0',4),         "slli s1,s0,4        # s1=ID*16 (byte offset)"))
    p.append((addi_enc('t2','zero',0x300),   "addi t2,zero,0x300  # t2=array base"))
    p.append((add_enc('s1','s1','t2'),        "add  s1,s1,t2       # s1=my chunk start"))
    return p

# ---- Workload 2: Read-Dominated ----
# All cores spin reading the shared variable at 0x100 in a tight loop.
# After the initial S-state fetch, reads are local cache hits with zero
# bus traffic. Demonstrates efficient silent sharing in MSI.
# Expected: low initial BusRd, near-zero everything else.
def workload2():
    return [
        (lw_enc('t0',0x100,'zero'),  "lw   t0,0x100(zero) # read shared variable"),
        (jal_enc('zero',-4),          "jal  zero,-4        # loop forever"),
    ]

# ---- Workload 3: High-Contention Write ----
# All 4 cores write the same address in a tight loop with no reads.
# Every write requires BusRdX (previous write invalidated local copy).
# Worst-case MSI coherence scenario - maximizes BusRdX and interventions.
# Expected: very high BusRdX and interventions, low BusRd/BusUpgr.
def workload3():
    return [
        (sw_enc('zero',0x100,'zero'), "sw   zero,0x100(zero) # write to shared"),
        (jal_enc('zero',-4),           "jal  zero,-4          # loop forever"),
    ]

# ---- Workload 4: Parallel Array Reduction ----
# Each core sums its 4-element chunk of a shared array, then accumulates
# into a shared result at 0x100. Combines private reads (S-state fast
# hits) with a shared write (coherence contention on accumulator).
# Array: 16 words at 0x300 initialized to 1..16.
# Each core owns 4 consecutive elements (core 0: [0..3], core 1: [4..7], etc.)
def workload4():
    h = id_and_chunk_header()
    main = len(h) * 4  # = 0x024
    p = h[:]
    p.append((lw_enc('s2',0,'s1'),          "lw   s2,0(s1)       # [main] load elem[0]"))
    p.append((lw_enc('t0',4,'s1'),           "lw   t0,4(s1)       # load elem[1]"))
    p.append((add_enc('s2','s2','t0'),        "add  s2,s2,t0       # sum+=e1"))
    p.append((lw_enc('t0',8,'s1'),           "lw   t0,8(s1)       # load elem[2]"))
    p.append((add_enc('s2','s2','t0'),        "add  s2,s2,t0       # sum+=e2"))
    p.append((lw_enc('t0',12,'s1'),          "lw   t0,12(s1)      # load elem[3]"))
    p.append((add_enc('s2','s2','t0'),        "add  s2,s2,t0       # sum+=e3"))
    p.append((lw_enc('t0',0x100,'zero'),     "lw   t0,0x100(zero) # load accumulator"))
    p.append((add_enc('t0','t0','s2'),        "add  t0,t0,s2       # acc+=partial_sum"))
    p.append((sw_enc('t0',0x100,'zero'),     "sw   t0,0x100(zero) # store accumulator"))
    jmp = main - len(p)*4
    p.append((jal_enc('zero',jmp),           f"jal  zero,{jmp}       # back to main"))
    return p

# ---- Workload 5: Stencil - Neighbor Update ----
# Each core reads its own first element and its right neighbor's first
# element (circular: core 3's right neighbor is core 0), adds them, and
# writes back. Generates cross-core sharing (BusRd on neighbor's M line)
# plus write invalidation (BusRdX on own line). Mimics 1D stencil kernels.
def workload5():
    h = id_and_chunk_header()
    loop = len(h) * 4  # = 0x024
    p = h[:]
    p.append((lw_enc('t0',0,'s1'),          "lw   t0,0(s1)       # [loop] load my[0]"))
    p.append((addi_enc('t3','s0',1),         "addi t3,s0,1        # right_id=my_id+1"))
    p.append((addi_enc('t4','zero',4),       "addi t4,zero,4      # t4=4"))
    # bne at offset len(p)*4. Target is 2 instructions further (+8 bytes).
    p.append((bne_enc('t3','t4',8),          "bne  t3,t4,+8       # skip wrap if !=4"))
    p.append((addi_enc('t3','zero',0),       "addi t3,zero,0      # wrap: right_id=0"))
    p.append((slli_enc('t4','t3',4),         "slli t4,t3,4        # t4=right_id*16"))
    p.append((addi_enc('t5','zero',0x300),   "addi t5,zero,0x300  # t5=array base"))
    p.append((add_enc('t4','t4','t5'),        "add  t4,t4,t5       # t4=neighbor addr"))
    p.append((lw_enc('t1',0,'t4'),           "lw   t1,0(t4)       # t1=neighbor[0]"))
    p.append((add_enc('t0','t0','t1'),        "add  t0,t0,t1       # t0=my[0]+nbr[0]"))
    p.append((sw_enc('t0',0,'s1'),           "sw   t0,0(s1)       # my[0]=t0"))
    jmp = loop - len(p)*4
    p.append((jal_enc('zero',jmp),           f"jal  zero,{jmp}       # back to loop"))
    return p

def print_init(prog, name, array_init=False):
    print(f"\n// {'='*65}")
    print(f"// {name}")
    print(f"// {len(prog)} instructions  ({len(prog)*4} bytes)")
    print(f"// {'='*65}")
    print("initial begin")
    for i,(enc,asm) in enumerate(prog):
        print(f"    memory[{i:2d}] = 32'h{enc:08x};  // {i*4:#06x}: {asm}")
    if array_init:
        print(f"    // Shared array: 16 words at 0x300 (word index 192-207)")
        print(f"    // Initialized to 1..16 so partial sums are nonzero")
        for i in range(16):
            print(f"    memory[{192+i}] = 32'd{i+1:2d};  // array[{i:2d}] = {i+1}")
    print("end")

print_init(workload2(), "Workload 2: Read-Dominated")
print_init(workload3(), "Workload 3: High-Contention Write")
print_init(workload4(), "Workload 4: Parallel Array Reduction", array_init=True)
print_init(workload5(), "Workload 5: Stencil - Neighbor Update", array_init=True)