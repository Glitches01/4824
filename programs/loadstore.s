main:
    li x1, 1
    li x2, 1
    li x4, 0x1000

    add x2, x2, x1
    sw x2, 0(x4)
    add x2, x2, 1
    sw x2, 0(x4)
    lw x2, 0(x4)
    wfi