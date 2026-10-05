"""The planner's cost on the robot's CPU, without the robot: runs the firmware
ELF's maze_plan_to/from on an emulated Cortex-M3 (unicorn) and counts
instructions, memory accesses and taken branches per planner pop (the cycles
are a model: check it against the robot's "decisiones en marcha" line).
Needs: pip install unicorn pyelftools. Maze files: github.com/micromouseonline/mazefiles.
usage: plan_cycles.py .pio/build/competition/firmware.elf [mazefile]"""
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR

elf = ELFFile(open(sys.argv[1], 'rb'))
syms = {}
for s in elf.get_section_by_name('.symtab').iter_symbols():
    if s['st_value']:
        syms.setdefault(s.name, s['st_value'])

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(0x08000000, 0x10000)
uc.mem_map(0x20000000, 0x5000)
STOP = 0x30000000
uc.mem_map(STOP, 0x1000)
for seg in elf.iter_segments():
    if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
        uc.mem_write(seg['p_vaddr'], seg.data())
        if seg['p_paddr'] != seg['p_vaddr']:
            uc.mem_write(seg['p_paddr'], seg.data())

stats = {'ins': 0, 'mem': 0, 'br': 0, 'next': None}
def on_code(u, addr, size, _):
    stats['ins'] += 1
    if stats['next'] is not None and addr != stats['next']:
        stats['br'] += 1
    stats['next'] = addr + size
def on_mem(u, access, addr, size, value, _):
    stats['mem'] += 1
uc.hook_add(UC_HOOK_CODE, on_code)
uc.hook_add(UC_HOOK_MEM_READ, on_mem)
uc.hook_add(UC_HOOK_MEM_WRITE, on_mem)

SP = 0x20004F00

def call(name, *args, stack=()):
    regs = [UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3]
    for r, v in zip(regs, args):
        uc.reg_write(r, v & 0xFFFFFFFF)
    for i, v in enumerate(stack):
        uc.mem_write(SP + 4 * i, (v & 0xFFFFFFFF).to_bytes(4, 'little'))
    uc.reg_write(UC_ARM_REG_SP, SP)
    uc.reg_write(UC_ARM_REG_LR, STOP | 1)
    for k in ('ins', 'mem', 'br'):
        stats[k] = 0
    stats['next'] = None
    uc.emu_start(syms[name] | 1, STOP)
    return dict(stats)

def pops():
    return int.from_bytes(uc.mem_read(syms['pops'], 4), 'little')

call('maze_init')
if len(sys.argv) > 2:   # a known maze: every wall blocked or crossed
    lines = [l.rstrip('\n') for l in open(sys.argv[2]) if l[:1] in 'o|+']
    for y in range(16):
        row, above = lines[2 * (15 - y) + 1], lines[2 * (15 - y)]
        for x in range(16):
            n = len(above) > 4 * x + 2 and above[4 * x + 2] != ' '
            e = len(row) > 4 * x + 4 and row[4 * x + 4] != ' '
            call('maze_mark_blocked' if n else 'maze_mark_crossed', x, y, 0)
            call('maze_mark_blocked' if e else 'maze_mark_crossed', x, y, 1)

targets = 0x20004000       # scratch RAM below the stack: a cellset (16 x uint16)
cost = syms['cost_a']
rows = [0] * 16
rows[7] = rows[8] = (1 << 7) | (1 << 8)
uc.mem_write(targets, b''.join(r.to_bytes(2, 'little') for r in rows))
for name, args, stack in (('maze_plan_to', (targets, 0, 2 | (1 << 8), cost), ()),
                          ('maze_plan_from', (0, 0, 0, 0), (2 | (1 << 8), cost))):
    p0 = pops()
    s = call(name, *args, stack=stack)
    n = pops() - p0
    # Cortex-M3, flash 2 wait states + prefetch: ~1 cycle an instruction, +1 a
    # data access, +3 a taken branch (pipeline refill from flash).
    cycles = s['ins'] + s['mem'] + 3 * s['br']
    print(f"{name}: {n} pops, {s['ins']} instr, {s['mem']} accesos, {s['br']} saltos -> "
          f"{s['ins'] / n:.0f} instr/pop, ~{cycles / n:.0f} ciclos/pop, ~{cycles / 72:.0f} us")
