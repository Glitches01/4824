/*
    TEST PROGRAM #1: copy memory contents of 16 elements starting at
                     address 0x1000 over to starting address 0x1100.


    long output[16];

    void
    main(void)
    {
      long i;
      *a = 0x1000;
          *b = 0x1100;

      for (i=0; i < 16; i++)
        {
          a[i] = i*10;
          b[i] = a[i];
        }
    }
    data = 0x1000
    li  x1, 1
    add  x2, x1, x0
    add  x3, x2, x1
    add  x4, x3, x2
    add  x5, x4, x3
    add  x6, x5, x4
    add  x7, x6, x5
    add  x8, x7, x6
    add  x9, x8, x7
    add  x10, x9, x8
    add  x11, x10, x9
    add  x12, x11, x10
    add  x13, x12, x11
    add  x14, x13, x12
    add  x15, x14, x13
    add  x16, x15, x14
    add  x17, x16, x15
    add  x18, x17, x16
    add  x19, x18, x17
    add  x20, x19, x18
    add  x21, x20, x19
    add  x22, x21, x20
    add  x23, x22, x21
    add  x24, x23, x22
    add  x25, x24, x23
    add  x26, x25, x24
    add  x27, x26, x25
    add  x28, x27, x26
    add  x29, x28, x27
    add  x30, x29, x28
    add  x31, x30, x29
    add  x1,  x31, x30
    add  x2,  x1, x31
    add  x3,  x2, x1
    add  x4, x3, x2
    add  x5, x4, x3
    add  x6, x5, x4
    add  x7, x6, x5
    add  x8, x7, x6
    add  x9, x8, x7
    add  x10, x9, x8
    add  x11, x10, x9
    add  x12, x11, x10
    add  x13, x12, x11
    add  x14, x13, x12
    add  x15, x14, x13
    add  x16, x15, x14
    add  x17, x16, x15
    li	x2, data
    li  x31, 0x0a
*/
    li x1, 1
    li x2, 1
    li x3, 1
    li x4, 16

loop: add	x3,	x2,	x2
    add x2, x2, x2
    add x1, x1, 1
    blt	x1,	x4,	loop

wfi
